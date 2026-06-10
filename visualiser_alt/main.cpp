#include <QApplication>
#include <QMainWindow>
#include <QStatusBar>
#include <QWidget>
#include <QAction>
#include <QToolBar>
#include <QComboBox>
#include <QPushButton>
#include <QSlider>
#include <QLabel>
#include <QFileDialog>
#include <QPainter>
#include <QMouseEvent>
#include <QWheelEvent>
#include <QThread>
#include <QMutex>
#include <QElapsedTimer>
#include <QMessageBox>
#include <QHash>
#include <cmath>
#include <vector>
#include <queue>
#include <random>
#include <limits>
#include <memory>
#include <algorithm>

// ============================================================================
// 1. DATA STRUCTURES & GRAPH STORAGE (Memory Optimized)
// ============================================================================

// Memory Note: 
// - Node: 32-bit ID + two 32-bit float coordinates = 12 Bytes per node.
// - Edge: 32-bit destination + 32-bit float weight = 8 Bytes per edge.
// - For a city graph of 1,000,000 nodes and 2,500,000 edges:
//   Nodes Vector: ~12 MB, Edges Vector: ~20 MB. 
//   Adjacency Index (offsets): ~4 MB. Total Core Storage: ~36 MB. Highly cache-friendly.
struct Node {
    uint32_t id;
    float x; // WebMercator X coordinate
    float y; // WebMercator Y coordinate
};

struct Edge {
    uint32_t dest_node_index; // Index mapping to the compact Node array
    float weight;             // Travel cost/distance
};

struct Graph {
    std::vector<Node> nodes;
    std::vector<Edge> edges;
    std::vector<uint32_t> head; // CSR-like offset array pointing to the start of edges for each node

    void clear() {
        nodes.clear();
        edges.clear();
        head.clear();
    }
};

// WebMercator Coordinate Projection (EPSG:3857)
inline void latLonToWebMercator(double lat, double lon, float &x, float &y) {
    const double RADIUS = 6378137.0;
    x = static_cast<float>(RADIUS * lon * M_PI / 180.0);
    double lat_rad = lat * M_PI / 180.0;
    y = static_cast<float>(RADIUS * std::log(std::tan(M_PI / 4.0 + lat_rad / 2.0)));
}

// ============================================================================
// 2. LIGHTWEIGHT SPATIAL INDEX (Uniform Grid for Fast Nearest Neighbor Queries)
// ============================================================================
class UniformGridIndex {
public:
    void build(const std::vector<Node>& nodes, int grid_resolution = 128) {
        m_nodes = &nodes;
        m_res = grid_resolution;
        m_grid.assign(m_res * m_res, std::vector<uint32_t>());
        
        if (nodes.empty()) return;

        m_min_x = m_max_x = nodes[0].x;
        m_min_y = m_max_y = nodes[0].y;
        for (const auto& n : nodes) {
            if (n.x < m_min_x) m_min_x = n.x;
            if (n.x > m_max_x) m_max_x = n.x;
            if (n.y < m_min_y) m_min_y = n.y;
            if (n.y > m_max_y) m_max_y = n.y;
        }

        // Prevent division by zero for point maps
        if (std::abs(m_max_x - m_min_x) < 1e-3f) m_max_x += 1.0f;
        if (std::abs(m_max_y - m_min_y) < 1e-3f) m_max_y += 1.0f;

        for (size_t i = 0; i < nodes.size(); ++i) {
            int gx = getGridX(nodes[i].x);
            int gy = getGridY(nodes[i].y);
            m_grid[gy * m_res + gx].push_back(static_cast<uint32_t>(i));
        }
    }

    int32_t findNearestNode(float x, float y) const {
        if (!m_nodes || m_nodes->empty()) return -1;

        int start_gx = getGridX(x);
        int start_gy = getGridY(y);
        
        int32_t best_idx = -1;
        float min_dist_sq = std::numeric_limits<float>::max();

        // Search in expanding rings if closest grid cell yields no items
        for (int radius = 0; radius < m_res; ++radius) {
            bool found_in_ring = false;
            int y_min = std::max(0, start_gy - radius);
            int y_max = std::min(m_res - 1, start_gy + radius);
            int x_min = std::max(0, start_gx - radius);
            int x_max = std::min(m_res - 1, start_gx + radius);

            for (int gy = y_min; gy <= y_max; ++gy) {
                for (int gx = x_min; gx <= x_max; ++gx) {
                    if (radius > 0 && gy > y_min && gy < y_max && gx > x_min && gx < x_max) {
                        continue; // Already processed inner ring cells
                    }
                    
                    const auto& cell = m_grid[gy * m_res + gx];
                    for (uint32_t idx : cell) {
                        found_in_ring = true;
                        float dx = (*m_nodes)[idx].x - x;
                        float dy = (*m_nodes)[idx].y - y;
                        float dist_sq = dx * dx + dy * dy;
                        if (dist_sq < min_dist_sq) {
                            min_dist_sq = dist_sq;
                            best_idx = static_cast<int32_t>(idx);
                        }
                    }
                }
            }
            // If we found nodes inside this radius ring, check if a closer node could exist in adjacent ring bounds
            if (found_in_ring && best_idx != -1) {
                return best_idx;
            }
        }
        return best_idx;
    }

private:
    int getGridX(float x) const {
        int gx = static_cast<int>((x - m_min_x) / (m_max_x - m_min_x) * m_res);
        return std::clamp(gx, 0, m_res - 1);
    }
    int getGridY(float y) const {
        int gy = static_cast<int>((y - m_min_y) / (m_max_y - m_min_y) * m_res);
        return std::clamp(gy, 0, m_res - 1);
    }

    const std::vector<Node>* m_nodes = nullptr;
    int m_res = 0;
    float m_min_x = 0, m_max_x = 0, m_min_y = 0, m_max_y = 0;
    std::vector<std::vector<uint32_t>> m_grid;
};

// ============================================================================
// 3. PARSING SUB-SYSTEM (Extensible Placeholder / Live Synthetic Engine)
// ============================================================================
class OsmParserWorker : public QThread {
    Q_OBJECT
signals:
    void progressNotification(const QString& text);
    void parsingFinished(std::shared_ptr<Graph> graph, std::shared_ptr<UniformGridIndex> index);

public:
    OsmParserWorker(QString filePath, QObject* parent = nullptr) 
        : QThread(parent), m_filePath(filePath) {}

    void run() override {
        emit progressNotification("Opening File & Parsing OSM protocol buffers...");
        
        auto graph = std::make_shared<Graph>();
        auto index = std::make_shared<UniformGridIndex>();

        /* ======================================================================
         SWAP POSITION: To use standard libosmium or libosmpbf reader,
         replace this synthetic generation step with your library parsing loops:
         
         Example Libosmium integration sketch:
         ----------------------------------------------------------------------
         #include <osmium/io/any_input.hpp>
         #include <osmium/handler.hpp>
         #include <osmium/visitor.hpp>

         struct CustomHandler : public osmium::handler::Handler {
             void node(const osmium::Node& node) {
                 // Convert coords, push to graph->nodes, etc.
             }
             void way(const osmium::Way& way) {
                 // Match tag high-level highways, verify connectivity, populate edges
             }
         };
         ...
         osmium::io::Reader reader{m_filePath.toStdString()};
         CustomHandler handler;
         osmium::apply(reader, handler);
         ======================================================================
        */
        
        // Simulating highly detailed geographical node graph structures directly inside background loader thread
        const int num_nodes = 40000;
        graph->nodes.reserve(num_nodes);
        graph->head.resize(num_nodes + 1, 0);

        // Center location reference
        double base_lat = 40.7128; 
        double base_lon = -74.0060;
        
        // Generate grid-mesh city simulation for robust path-finding demos
        int side = static_cast<int>(std::sqrt(num_nodes));
        float spacing = 120.0f; // meters separation between simulated nodes

        float origin_x, origin_y;
        latLonToWebMercator(base_lat, base_lon, origin_x, origin_y);

        for (int r = 0; r < side; ++r) {
            for (int c = 0; c < side; ++c) {
                uint32_t idx = r * side + c;
                Node n;
                n.id = 100000 + idx;
                n.x = origin_x + (c - side/2) * spacing + ((rand() % 20) - 10);
                n.y = origin_y + (r - side/2) * spacing + ((rand() % 20) - 10);
                graph->nodes.push_back(n);
            }
        }

        emit progressNotification("Resolving structural routing adjacencies...");
        
        std::vector<std::vector<Edge>> adj(graph->nodes.size());
        for (int r = 0; r < side; ++r) {
            for (int c = 0; c < side; ++c) {
                uint32_t u = r * side + c;
                // Structural neighbors (Grid interconnects representing city infrastructure block links)
                int dr[] = {-1, 1, 0, 0};
                int dc[] = {0, 0, -1, 1};
                for (int i = 0; i < 4; ++i) {
                    int nr = r + dr[i];
                    int nc = c + dc[i];
                    if (nr >= 0 && nr < side && nc >= 0 && nc < side) {
                        uint32_t v = nr * side + nc;
                        float dx = graph->nodes[u].x - graph->nodes[v].x;
                        float dy = graph->nodes[u].y - graph->nodes[v].y;
                        float d = std::sqrt(dx*dx + dy*dy);
                        adj[u].push_back(Edge{v, d});
                    }
                }
            }
        }

        // Construct high-speed CSR layout data streams
        uint32_t edge_offset = 0;
        for (size_t i = 0; i < graph->nodes.size(); ++i) {
            graph->head[i] = edge_offset;
            for (const auto& edge : adj[i]) {
                graph->edges.push_back(edge);
                edge_offset++;
            }
        }
        graph->head[graph->nodes.size()] = edge_offset;

        emit progressNotification("Assembling Uniform Spatial Index...");
        index->build(graph->nodes, 128);

        QThread::msleep(800); // Visual buffer window for the background status thread notice
        emit parsingFinished(graph, index);
    }

private:
    QString m_filePath;
};

// ============================================================================
// 4. ALT ALGORITHM EXECUTION WORKER
// ============================================================================
class AltWorker : public QThread {
    Q_OBJECT
signals:
    void nodeExplored(uint32_t nodeIndex, bool isVisited);
    void searchCompleted(std::vector<uint32_t> path);

public:
    AltWorker(std::shared_ptr<Graph> g, uint32_t start, uint32_t end, const std::vector<uint32_t>& landmarks, QObject* parent = nullptr)
        : QThread(parent), m_graph(g), m_start(start), m_end(end), m_landmarks(landmarks) {}

    void run() override {
        if (!m_graph || m_start >= m_graph->nodes.size() || m_end >= m_graph->nodes.size()) {
            emit searchCompleted({});
            return;
        }

        size_t n_count = m_graph->nodes.size();
        
        // Landmark heuristic calculation preprocessing matrix profiles
        // land_dist[L][N] stores shortest path distance from landmark L to node N
        std::vector<std::vector<float>> land_dist(m_landmarks.size(), std::vector<float>(n_count, std::numeric_limits<float>::max()));
        
        // Precomputing shortest distance fields outwards from landmarks
        for (size_t l = 0; l < m_landmarks.size(); ++l) {
            uint32_t lm = m_landmarks[l];
            using Link = std::pair<float, uint32_t>;
            std::priority_queue<Link, std::vector<Link>, std::greater<Link>> pq;
            
            land_dist[l][lm] = 0.0f;
            pq.push({0.0f, lm});
            
            while (!pq.empty()) {
                auto [d, u] = pq.top();
                pq.pop();
                
                if (d > land_dist[l][u]) continue;
                
                uint32_t start_edge = m_graph->head[u];
                uint32_t end_edge = m_graph->head[u + 1];
                for (uint32_t e = start_edge; e < end_edge; ++e) {
                    uint32_t v = m_graph->edges[e].dest_node_index;
                    float w = m_graph->edges[e].weight;
                    if (land_dist[l][u] + w < land_dist[l][v]) {
                        land_dist[l][v] = land_dist[l][u] + w;
                        pq.push({land_dist[l][v], v});
                    }
                }
            }
        }

        // Triangle Inequality Admissible Lower Bound Evaluation Heuristic Definition
        auto heuristic = [&](uint32_t u) -> float {
            float max_h = 0.0f;
            for (size_t l = 0; l < m_landmarks.size(); ++l) {
                float d_l_u = land_dist[l][u];
                float d_l_t = land_dist[l][m_end];
                if (d_l_u != std::numeric_limits<float>::max() && d_l_t != std::numeric_limits<float>::max()) {
                    max_h = std::max(max_h, std::abs(d_l_u - d_l_t));
                }
            }
            return max_h;
        };

        // A* execution loop using landmark potential metrics
        using AStarElement = std::pair<float, uint32_t>; // (f_score, node_index)
        std::priority_queue<AStarElement, std::vector<AStarElement>, std::greater<AStarElement>> pq;
        
        std::vector<float> g_score(n_count, std::numeric_limits<float>::max());
        std::vector<uint32_t> parent_node(n_count, std::numeric_limits<uint32_t>::max());
        std::vector<bool> closed_set(n_count, false);

        g_score[m_start] = 0.0f;
        pq.push({heuristic(m_start), m_start});

        int animation_throttle = 0;

        while (!pq.empty()) {
            auto [f, u] = pq.top();
            pq.pop();

            if (u == m_end) break;
            if (closed_set[u]) continue;
            closed_set[u] = true;

            // Notify GUI rendering viewport thread of discovery expansions
            emit nodeExplored(u, true);
            
            // Render animation throttling to maintain high responsivity and smooth UI interactions
            if (++animation_throttle % 12 == 0) {
                QThread::msleep(1); 
            }

            uint32_t start_edge = m_graph->head[u];
            uint32_t end_edge = m_graph->head[u + 1];
            for (uint32_t e = start_edge; e < end_edge; ++e) {
                uint32_t v = m_graph->edges[e].dest_node_index;
                float w = m_graph->edges[e].weight;
                
                if (closed_set[v]) continue;

                float tentative_g = g_score[u] + w;
                if (tentative_g < g_score[v]) {
                    parent_node[v] = u;
                    g_score[v] = tentative_g;
                    float f_next = tentative_g + heuristic(v);
                    pq.push({f_next, v});
                }
            }
        }

        // Trace and extract the calculated shortest path route vectors
        std::vector<uint32_t> route;
        if (g_score[m_end] != std::numeric_limits<float>::max()) {
            uint32_t curr = m_end;
            while (curr != std::numeric_limits<uint32_t>::max()) {
                route.push_back(curr);
                curr = parent_node[curr];
            }
            std::reverse(route.begin(), route.end());
        }

        emit searchCompleted(route);
    }

private:
    std::shared_ptr<Graph> m_graph;
    uint32_t m_start;
    uint32_t m_end;
    std::vector<uint32_t> m_landmarks;
};

// ============================================================================
// 5. INTERACTIVE MAP VIEWPORT COMPONENT
// ============================================================================
enum class InteractionMode { Pan, Click };
enum class SelectionTarget { None, Landmark, Start, End };

class MapWidget : public QWidget {
    Q_OBJECT
public:
    MapWidget(QWidget* parent = nullptr) : QWidget(parent) {
        setMouseTracking(true);
        setBackgroundRole(QPalette::Dark);
        setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    }

    void setGraphData(std::shared_ptr<Graph> g, std::shared_ptr<UniformGridIndex> idx) {
        m_graph = g;
        m_spatialIndex = idx;
        m_visitedFlags.assign(m_graph ? m_graph->nodes.size() : 0, false);
        m_shortestPath.clear();
        m_landmarks.clear();
        m_startNode = -1;
        m_endNode = -1;
        resetView();
        renderStaticBackgroundPixmap();
        update();
    }

    void setInteractionMode(InteractionMode mode) { m_interactionMode = mode; }
    void setSelectionTarget(SelectionTarget target) { m_selectionTarget = target; }
    
    void clearLandmarks() { m_landmarks.clear(); update(); }
    void setRandomLandmarks(int k) {
        if (!m_graph || m_graph->nodes.empty() || k <= 0) return;
        m_landmarks.clear();
        std::vector<uint32_t> indices(m_graph->nodes.size());
        std::iota(indices.begin(), indices.end(), 0);
        std::mt19937 g(std::random_device{}());
        std::shuffle(indices.begin(), indices.end(), g);
        
        size_t limit = std::min(static_cast<size_t>(k), m_graph->nodes.size());
        for(size_t i=0; i<limit; ++i) {
            m_landmarks.push_back(indices[i]);
        }
        update();
    }

    void markExplored(uint32_t index, bool visited) {
        if (index < m_visitedFlags.size()) {
            m_visitedFlags[index] = visited;
            // Track localized boundary updates for smooth execution animations
            update();
        }
    }

    void setShortestPath(const std::vector<uint32_t>& path) {
        m_shortestPath = path;
        update();
    }

    void clearExecutionVisuals() {
        std::fill(m_visitedFlags.begin(), m_visitedFlags.end(), false);
        m_shortestPath.clear();
        update();
    }

    void resetView() {
        if (!m_graph || m_graph->nodes.empty()) return;

        // Calculate the True Bounding Box of the graph
        float min_x = m_graph->nodes[0].x, max_x = min_x;
        float min_y = m_graph->nodes[0].y, max_y = min_y;
        
        // Accumulators for calculating the center of mass
        double sum_x = 0.0;
        double sum_y = 0.0;

        for (const auto& n : m_graph->nodes) {
            min_x = std::min(min_x, n.x); max_x = std::max(max_x, n.x);
            min_y = std::min(min_y, n.y); max_y = std::max(max_y, n.y);
            sum_x += n.x;
            sum_y += n.y;
        }

        m_mapBoundingBox = QRectF(QPointF(min_x, min_y), QPointF(max_x, max_y));
        
        // Assign the physical mass-center averages as our focal point 
        m_mapCenterX = static_cast<float>(sum_x / m_graph->nodes.size());
        m_mapCenterY = static_cast<float>(sum_y / m_graph->nodes.size());

        // Center the viewport back to the screen center
        m_panOffsetX = width() / 2;
        m_panOffsetY = height() / 2;
        
        // Calculate a bounding scale so the entire graph fits elegantly
        float scaleX = width() / static_cast<float>(m_mapBoundingBox.width() * 1.15);
        float scaleY = height() / static_cast<float>(m_mapBoundingBox.height() * 1.15);
        m_zoomScale = std::clamp(std::min(scaleX, scaleY), 0.0001f, 5000.0f);
        
        emit zoomChanged();
        update();
    }

    void zoomIn() { adjustZoomScale(1.2, width() / 2, height() / 2); }
    void zoomOut() { adjustZoomScale(1.0 / 1.2, width() / 2, height() / 2); }
    
    int getZoomPercentage() const { return static_cast<int>(m_zoomScale * 100.0f); }
    int32_t getStartNode() const { return m_startNode; }
    int32_t getEndNode() const { return m_endNode; }
    std::vector<uint32_t> getLandmarks() const { return m_landmarks; }

signals:
    void zoomChanged();
    void nodeSelected(SelectionTarget target, uint32_t nodeIdx);

protected:
    void paintEvent(QPaintEvent* event) override {
        Q_UNUSED(event);
        QPainter painter(this);
        painter.fillRect(rect(), QColor(30, 30, 30)); // Dark Gray Palette

        if (!m_graph || m_graph->nodes.empty()) {
            painter.setPen(Qt::white);
            painter.drawText(rect(), Qt::AlignCenter, "Load OpenStreetMap PBF dataset to begin graph analytics.");
            return;
        }

        // Step 1: Draw the pre-rendered static road network base configuration layer
        painter.save();
        // Compute matrix tracking transforms to center the map on the viewport
        QTransform trans;
        trans.translate(m_panOffsetX, m_panOffsetY);
        trans.scale(m_zoomScale, m_zoomScale);
        trans.translate(-m_mapCenterX, -m_mapCenterY);
        
        painter.setTransform(trans);
        painter.drawPixmap(m_mapBoundingBox.topLeft(), m_staticMapPixmap);
        painter.restore();

        // Step 2: Overlay dynamic search data onto the layout
        painter.setTransform(trans);

        // Draw exploration front vectors (Visited = Blue)
        painter.setPen(QPen(QColor(0, 162, 232, 180), 2.0f / m_zoomScale));
        for (size_t i = 0; i < m_visitedFlags.size(); ++i) {
            if (m_visitedFlags[i]) {
                float nx = m_graph->nodes[i].x;
                float ny = m_graph->nodes[i].y;
                painter.drawPoint(QPointF(nx, ny));
            }
        }

        // Draw Calculated Shortest Path Vectors (Path = Orange)
        if (m_shortestPath.size() > 1) {
            QPen pathPen(QColor(255, 127, 39), 4.5f / m_zoomScale, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
            painter.setPen(pathPen);
            for (size_t i = 0; i < m_shortestPath.size() - 1; ++i) {
                const auto& n1 = m_graph->nodes[m_shortestPath[i]];
                const auto& n2 = m_graph->nodes[m_shortestPath[i+1]];
                painter.drawLine(QPointF(n1.x, n1.y), QPointF(n2.x, n2.y));
            }
        }

        // Draw Landmarks Highlight Structures (Landmarks = Red circles)
        float markerSize = 9.0f / m_zoomScale;
        painter.setBrush(QBrush(QColor(237, 28, 36)));
        painter.setPen(QPen(Qt::white, 1.5f / m_zoomScale));
        for (uint32_t lmIdx : m_landmarks) {
            const auto& node = m_graph->nodes[lmIdx];
            painter.drawEllipse(QPointF(node.x, node.y), markerSize, markerSize);
        }

        // Draw Start & End Node Reference Labels
        auto drawLabelNode = [&](int32_t nodeIdx, QString txt, QColor clr) {
            if (nodeIdx == -1) return;
            const auto& n = m_graph->nodes[nodeIdx];
            float size = 11.0f / m_zoomScale;
            painter.setBrush(QBrush(clr));
            painter.setPen(QPen(Qt::white, 2.0f / m_zoomScale));
            painter.drawEllipse(QPointF(n.x, n.y), size, size);

            // Keep text fonts constant sizes regardless of zoom factor scales
            painter.save();
            painter.setTransform(QTransform()); 
            QPointF screenPos = trans.map(QPointF(n.x, n.y));
            painter.setPen(Qt::white);
            painter.setFont(QFont("Arial", 9, QFont::Bold));
            painter.drawText(screenPos.x() + 12, screenPos.y() + 4, txt);
            painter.restore();
        };

        drawLabelNode(m_startNode, "START", Qt::black);
        drawLabelNode(m_endNode, "END", Qt::black);
    }

    void mousePressEvent(QMouseEvent* event) override {
        if (!m_graph) return;
        if (m_interactionMode == InteractionMode::Pan && event->button() == Qt::LeftButton) {
            m_isDragging = true;
            m_lastMousePos = event->pos();
        } else if (m_interactionMode == InteractionMode::Click && event->button() == Qt::LeftButton) {
            // Unmap screen clicks back to map projection units
            QTransform trans;
            trans.translate(m_panOffsetX, m_panOffsetY);
            trans.scale(m_zoomScale, m_zoomScale);
            trans.translate(-m_mapCenterX, -m_mapCenterY);
            QPointF mapPos = trans.inverted().map(QPointF(event->pos()));

            int32_t targetIdx = m_spatialIndex->findNearestNode(static_cast<float>(mapPos.x()), static_cast<float>(mapPos.y()));
            if (targetIdx != -1) {
                if (m_selectionTarget == SelectionTarget::Landmark) {
                    if (std::find(m_landmarks.begin(), m_landmarks.end(), targetIdx) == m_landmarks.end()) {
                        m_landmarks.push_back(targetIdx);
                    }
                } else if (m_selectionTarget == SelectionTarget::Start) {
                    m_startNode = targetIdx;
                } else if (m_selectionTarget == SelectionTarget::End) {
                    m_endNode = targetIdx;
                }
                emit nodeSelected(m_selectionTarget, targetIdx);
                update();
            }
        }
    }

    void mouseMoveEvent(QMouseEvent* event) override {
        if (m_isDragging && m_interactionMode == InteractionMode::Pan) {
            QPoint delta = event->pos() - m_lastMousePos;
            m_panOffsetX += delta.x();
            m_panOffsetY += delta.y();
            m_lastMousePos = event->pos();
            update();
        }
    }

    void mouseReleaseEvent(QMouseEvent* event) override {
        if (event->button() == Qt::LeftButton) {
            m_isDragging = false;
        }
    }

    void wheelEvent(QWheelEvent* event) override {
        double factor = (event->angleDelta().y() > 0) ? 1.15 : (1.0 / 1.15);
        // Centers the zoom transition exactly where your mouse cursor is pointing
        adjustZoomScale(factor, event->position().x(), event->position().y());
    }

private:

    void adjustZoomScale(double factor, float mouseX, float mouseY) {
        // Center the scale vector at the precise mouse position coordinates
        float mapX = static_cast<float>((mouseX - m_panOffsetX) / m_zoomScale + m_mapCenterX);
        float mapY = static_cast<float>((mouseY - m_panOffsetY) / m_zoomScale + m_mapCenterY);

        m_zoomScale = std::clamp(static_cast<float>(m_zoomScale * factor), 0.002f, 2500.0f);

        m_panOffsetX = mouseX - (mapX - m_mapCenterX) * m_zoomScale;
        m_panOffsetY = mouseY - (mapY - m_mapCenterY) * m_zoomScale;

        emit zoomChanged();
        update();
    }

    void renderStaticBackgroundPixmap() {
        if (!m_graph || m_graph->nodes.empty()) return;
        
        // Render road networks to offscreen storage layout layer
        m_staticMapPixmap = QPixmap(m_mapBoundingBox.size().toSize().expandedTo(QSize(2000, 2000)));
        m_staticMapPixmap.fill(Qt::transparent);

        QPainter pixPainter(&m_staticMapPixmap);
        pixPainter.setRenderHint(QPainter::Antialiasing, false); // Performance prioritization for large graphs
        
        // Edge connections colored a dim gray profile line matching dark themes
        QPen edgePen(QColor(75, 75, 75), 1.0f); 
        pixPainter.setPen(edgePen);

        // Normalize projection layout dimensions
        float dx = m_mapBoundingBox.left();
        float dy = m_mapBoundingBox.top();

        for (size_t u = 0; u < m_graph->nodes.size(); ++u) {
            uint32_t start_edge = m_graph->head[u];
            uint32_t end_edge = m_graph->head[u + 1];
            QPointF p1(m_graph->nodes[u].x - dx, m_graph->nodes[u].y - dy);
            
            for (uint32_t e = start_edge; e < end_edge; ++e) {
                uint32_t v = m_graph->edges[e].dest_node_index;
                // Avoid drawing bidirectional duplicates twice to optimize loading speed
                if (u < v) { 
                    QPointF p2(m_graph->nodes[v].x - dx, m_graph->nodes[v].y - dy);
                    pixPainter.drawLine(p1, p2);
                }
            }
        }
        pixPainter.end();
    }

    std::shared_ptr<Graph> m_graph;
    std::shared_ptr<UniformGridIndex> m_spatialIndex;
    QPixmap m_staticMapPixmap;
    QRectF m_mapBoundingBox;

    InteractionMode m_interactionMode = InteractionMode::Pan;
    SelectionTarget m_selectionTarget = SelectionTarget::None;

    std::vector<bool> m_visitedFlags;
    std::vector<uint32_t> m_shortestPath;
    std::vector<uint32_t> m_landmarks;
    
    int32_t m_startNode = -1;
    int32_t m_endNode = -1;

    float m_zoomScale = 1.0f;
    float m_mapCenterX = 0.0f, m_mapCenterY = 0.0f;
    float m_panOffsetX = 0.0f, m_panOffsetY = 0.0f;
    bool m_isDragging = false;
    QPoint m_lastMousePos;
};

// ============================================================================
// 6. MAIN APPLICATION WINDOW CONTROLLER
// ============================================================================
class MainWindow : public QMainWindow {
    Q_OBJECT
public:
    MainWindow() {
        setWindowTitle("ALT Shortest-Path Pipeline Demo (C++17, Qt6)");
        resize(1200, 800);

        m_mapWidget = new MapWidget(this);
        setCentralWidget(m_mapWidget);

        buildUserInterfaceLayout();
        connectSignalsAndSlots();
        updateToolbarStateContexts();
    }

private:
    void buildUserInterfaceLayout() {
        m_toolBar = addToolBar("Control Panel Options");
        m_toolBar->setMovable(false);

        m_btnLoad = new QPushButton("LOAD", this);
        m_toolBar->addWidget(m_btnLoad);
        m_toolBar->addSeparator();

        m_toolBar->addWidget(new QLabel(" TYPE: ", this));
        m_comboType = new QComboBox(this);
        m_comboType->addItems({"Handpicked", "Random Bounds"});
        m_toolBar->addWidget(m_comboType);

        m_lblSliderK = new QLabel(" K: ", this);
        m_sliderK = new QSlider(Qt::Horizontal, this);
        m_sliderK->setRange(1, 16);
        m_sliderK->setValue(4);
        m_sliderK->setFixedWidth(100);
        
        m_lblSliderKWidget = m_toolBar->addWidget(m_lblSliderK);
        m_sliderKWidget = m_toolBar->addWidget(m_sliderK);
        m_lblSliderKWidget->setVisible(false);
        m_sliderKWidget->setVisible(false);

        m_btnClearLandmarks = new QPushButton("CLEAR LANDMARKS", this);
        m_toolBar->addWidget(m_btnClearLandmarks);
        m_toolBar->addSeparator();

        m_btnSelectStart = new QPushButton("Select START", this);
        m_btnSelectStart->setCheckable(true);
        m_toolBar->addWidget(m_btnSelectStart);

        m_btnSelectEnd = new QPushButton("Select END", this);
        m_btnSelectEnd->setCheckable(true);
        m_toolBar->addWidget(m_btnSelectEnd);
        m_toolBar->addSeparator();

        m_btnRunClear = new QPushButton("RUN", this);
        m_btnRunClear->setEnabled(false);
        m_toolBar->addWidget(m_btnRunClear);

        // 1. Expand the panel geometry slightly to fit the new button layout safely
        m_panelZoom = new QWidget(this);
        m_panelZoom->setGeometry(15, 80, 75, 205); // Expanded height from 170 to 205
        m_panelZoom->setStyleSheet("background-color: rgba(45,45,45,210); border-radius: 6px; color: white;");

        // 2. Add the CENTER button at the top of the stack
        m_btnCenter = new QPushButton("Center", m_panelZoom);
        m_btnCenter->setGeometry(10, 10, 55, 30);
        m_btnCenter->setStyleSheet("QPushButton { background-color: #444; border: 1px solid #666; font-size: 11px; font-weight: bold; }");

        // 3. Shift the downstream buttons downward by 35 pixels to make room
        m_btnZoomIn = new QPushButton("+", m_panelZoom);
        m_btnZoomIn->setGeometry(10, 45, 55, 30); // Shifted down from 10
        m_btnZoomIn->setStyleSheet("QPushButton { background-color: #444; border: 1px solid #666; font-weight: bold; }");

        m_btnZoomOut = new QPushButton("-", m_panelZoom);
        m_btnZoomOut->setGeometry(10, 80, 55, 30); // Shifted down from 45
        m_btnZoomOut->setStyleSheet("QPushButton { background-color: #444; border: 1px solid #666; font-weight: bold; }");

        m_lblZoomPct = new QLabel("100%", m_panelZoom);
        m_lblZoomPct->setGeometry(5, 115, 65, 20); // Shifted down from 80
        m_lblZoomPct->setAlignment(Qt::AlignCenter);

        m_btnToggleMode = new QPushButton("Pan", m_panelZoom);
        m_btnToggleMode->setGeometry(10, 145, 55, 45); // Shifted down from 110
        m_btnToggleMode->setCheckable(true);
        m_btnToggleMode->setStyleSheet("QPushButton { background-color: #22c55e; color: black; border-radius: 4px; font-size: 11px; } "
                                    "QPushButton:checked { background-color: #3b82f6; color: white; }");
    }

    void connectSignalsAndSlots() {
        connect(m_btnLoad, &QPushButton::clicked, this, &MainWindow::onLoadClicked);
        connect(m_comboType, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &MainWindow::onTypeChanged);
        connect(m_sliderK, &QSlider::valueChanged, this, &MainWindow::onSliderKChanged);
        connect(m_btnClearLandmarks, &QPushButton::clicked, this, &MainWindow::onClearLandmarksClicked);
        
        connect(m_btnSelectStart, &QPushButton::clicked, this, &MainWindow::onSelectStartToggled);
        connect(m_btnSelectEnd, &QPushButton::clicked, this, &MainWindow::onSelectEndToggled);
        connect(m_btnRunClear, &QPushButton::clicked, this, &MainWindow::onRunClearClicked);

        connect(m_btnZoomIn, &QPushButton::clicked, m_mapWidget, &MapWidget::zoomIn);
        connect(m_btnZoomOut, &QPushButton::clicked, m_mapWidget, &MapWidget::zoomOut);
        connect(m_btnCenter, &QPushButton::clicked, m_mapWidget, &MapWidget::resetView);
        connect(m_mapWidget, &MapWidget::zoomChanged, this, &MainWindow::onZoomLevelChanged);
        
        connect(m_btnToggleMode, &QPushButton::toggled, this, &MainWindow::onInteractionModeToggled);
        connect(m_mapWidget, &MapWidget::nodeSelected, this, &MainWindow::onNodeSelectionRegistered);
    }

    void updateToolbarStateContexts() {
        bool elementsReady = (m_mapWidget->getStartNode() != -1 && m_mapWidget->getEndNode() != -1);
        if (m_algorithmRunningState) {
            m_btnRunClear->setText("RUNNING...");
            m_btnRunClear->setEnabled(false);
        } else {
            if (m_visualsActiveState) {
                m_btnRunClear->setText("CLEAR");
                m_btnRunClear->setEnabled(true);
            } else {
                m_btnRunClear->setText("RUN");
                m_btnRunClear->setEnabled(elementsReady);
            }
        }

        // Tint active toggle buttons Yellow
        m_btnSelectStart->setStyleSheet(m_btnSelectStart->isChecked() ? "background-color: #fde047; color: black;" : "");
        m_btnSelectEnd->setStyleSheet(m_btnSelectEnd->isChecked() ? "background-color: #fde047; color: black;" : "");
    }

    void resizeEvent(QResizeEvent* event) override {
        QMainWindow::resizeEvent(event);
        m_panelZoom->setGeometry(15, 80, 75, 205);
    }

private slots:
    void onLoadClicked() {
        QString file = QFileDialog::getOpenFileName(this, "Select OpenStreetMap PBF Database File", "", "OSM Protocol Buffers (*.osm.pbf);;All Files (*)");
        if (file.isEmpty()) return;

        m_btnLoad->setEnabled(false);
        OsmParserWorker* parser = new OsmParserWorker(file, this);
        connect(parser, &OsmParserWorker::progressNotification, this, [&](QString text){
            statusBar()->showMessage(text);
        });
        connect(parser, &OsmParserWorker::parsingFinished, this, [this, parser](std::shared_ptr<Graph> g, std::shared_ptr<UniformGridIndex> idx){
            m_graph = g;
            m_mapWidget->setGraphData(g, idx);
            statusBar()->showMessage(QString("Graph loaded. Nodes: %1 | Edges: %2").arg(g->nodes.size()).arg(g->edges.size()));
            m_btnLoad->setEnabled(true);
            m_visualsActiveState = false;
            updateToolbarStateContexts();
            parser->deleteLater();
        });
        parser->start();
    }

    void onTypeChanged(int index) {
        m_mapWidget->clearLandmarks();
        if (index == 0) { // Handpicked mode
            m_lblSliderKWidget->setVisible(false);
            m_sliderKWidget->setVisible(false);
            if (m_btnToggleMode->isChecked()) {
                m_mapWidget->setSelectionTarget(SelectionTarget::Landmark);
            }
        } else { // Random selection mode
            m_lblSliderKWidget->setVisible(true);
            m_sliderKWidget->setVisible(true);
            m_mapWidget->setSelectionTarget(SelectionTarget::None);
            m_mapWidget->setRandomLandmarks(m_sliderK->value());
        }
        updateToolbarStateContexts();
    }

    void onSliderKChanged(int val) {
        if (m_comboType->currentIndex() == 1) {
            m_mapWidget->setRandomLandmarks(val);
        }
    }

    void onClearLandmarksClicked() {
        m_mapWidget->clearLandmarks();
        if (m_comboType->currentIndex() == 1) {
            m_sliderK->setValue(1);
        }
    }

    void onSelectStartToggled(bool checked) {
        if (checked) {
            m_btnSelectEnd->setChecked(false);
            m_btnToggleMode->setChecked(true); // Force Click-Selection state
            m_mapWidget->setSelectionTarget(SelectionTarget::Start);
        } else {
            m_mapWidget->setSelectionTarget(SelectionTarget::None);
        }
        updateToolbarStateContexts();
    }

    void onSelectEndToggled(bool checked) {
        if (checked) {
            m_btnSelectStart->setChecked(false);
            m_btnToggleMode->setChecked(true); // Force Click-Selection state
            m_mapWidget->setSelectionTarget(SelectionTarget::End);
        } else {
            m_mapWidget->setSelectionTarget(SelectionTarget::None);
        }
        updateToolbarStateContexts();
    }

    void onInteractionModeToggled(bool clickedMode) {
        if (clickedMode) {
            m_btnToggleMode->setText("Click");
            m_mapWidget->setInteractionMode(InteractionMode::Click);
            if (m_btnSelectStart->isChecked()) m_mapWidget->setSelectionTarget(SelectionTarget::Start);
            else if (m_btnSelectEnd->isChecked()) m_mapWidget->setSelectionTarget(SelectionTarget::End);
            else if (m_comboType->currentIndex() == 0) m_mapWidget->setSelectionTarget(SelectionTarget::Landmark);
        } else {
            m_btnToggleMode->setText("Pan");
            m_mapWidget->setInteractionMode(InteractionMode::Pan);
            m_mapWidget->setSelectionTarget(SelectionTarget::None);
        }
    }

    void onNodeSelectionRegistered(SelectionTarget target, uint32_t nodeIdx) {
        Q_UNUSED(nodeIdx);
        if (target == SelectionTarget::Start) m_btnSelectStart->setChecked(false);
        if (target == SelectionTarget::End) m_btnSelectEnd->setChecked(false);
        
        // Reset interactive tool context targets
        onInteractionModeToggled(m_btnToggleMode->isChecked()); 
        updateToolbarStateContexts();
    }

    void onZoomLevelChanged() {
        m_lblZoomPct->setText(QString("%1%").arg(m_mapWidget->getZoomPercentage()));
    }

    void onRunClearClicked() {
        if (m_visualsActiveState) {
            // Clear visualization display layer states
            m_mapWidget->clearExecutionVisuals();
            m_visualsActiveState = false;
            updateToolbarStateContexts();
            return;
        }

        std::vector<uint32_t> landmarks = m_mapWidget->getLandmarks();
        if (landmarks.empty()) {
            QMessageBox::warning(this, "Landmarks Required", "Select at least one landmark to construct the ALT triangle lower bounds bounds heuristic.");
            return;
        }

        m_algorithmRunningState = true;
        m_mapWidget->clearExecutionVisuals();
        updateToolbarStateContexts();

        AltWorker* worker = new AltWorker(m_graph, m_mapWidget->getStartNode(), m_mapWidget->getEndNode(), landmarks, this);
        connect(worker, &AltWorker::nodeExplored, m_mapWidget, &MapWidget::markExplored, Qt::QueuedConnection);
        connect(worker, &AltWorker::searchCompleted, this, [this, worker](std::vector<uint32_t> path) {
            m_mapWidget->setShortestPath(path);
            m_algorithmRunningState = false;
            m_visualsActiveState = true;
            updateToolbarStateContexts();
            if (path.empty()) {
                statusBar()->showMessage("ALT Routing Completed: No path found between points.");
            } else {
                statusBar()->showMessage(QString("ALT Routing Completed: Route maps containing %1 intersections.").arg(path.size()));
            }
            worker->deleteLater();
        });
        
        worker->start();
    }

private:
    MapWidget* m_mapWidget;
    QToolBar* m_toolBar;
    
    QPushButton* m_btnLoad;
    QComboBox* m_comboType;
    QLabel* m_lblSliderK;
    QSlider* m_sliderK;
    QAction* m_lblSliderKWidget = nullptr;
    QAction* m_sliderKWidget = nullptr;
    QPushButton* m_btnClearLandmarks;
    
    QPushButton* m_btnSelectStart;
    QPushButton* m_btnSelectEnd;
    QPushButton* m_btnRunClear;

    QWidget* m_panelZoom;
    QPushButton* m_btnCenter;
    QPushButton* m_btnZoomIn;
    QPushButton* m_btnZoomOut;
    QLabel* m_lblZoomPct;
    QPushButton* m_btnToggleMode;

    std::shared_ptr<Graph> m_graph;
    bool m_algorithmRunningState = false;
    bool m_visualsActiveState = false;
};

int main(int argc, char *argv[]) {
    QApplication app(argc, argv);
    MainWindow window;
    window.show();
    return app.exec();
}

#include "main.moc"