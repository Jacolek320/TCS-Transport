#include "mainwindow.h"
#include "algo.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFileDialog>
#include <QLabel>
#include <QMouseEvent>
#include <iostream>

// GraphView Implementation
void GraphView::mousePressEvent(QMouseEvent* ev) {
    emit sceneClicked(mapToScene(ev->pos()));
    QGraphicsView::mousePressEvent(ev);
}

// MainWindow Implementation
MainWindow::MainWindow() {
    setWindowTitle("Path Visualizer");
    auto* mainL = new QVBoxLayout(this);
    auto* controls = new QHBoxLayout();

    auto* loadBtn = new QPushButton("Load .osm.pbf");
    auto* startBtn = new QPushButton("START");
    auto* pauseBtn = new QPushButton("PAUSE");
    auto* resetBtn = new QPushButton("RESET");
    speedSlider = new QSlider(Qt::Horizontal);
    speedSlider->setRange(1, 100);
    speedSlider->setValue(50);

    controls->addWidget(loadBtn);
    controls->addWidget(startBtn);
    controls->addWidget(pauseBtn);
    controls->addWidget(resetBtn);
    controls->addWidget(new QLabel("Speed"));
    controls->addWidget(speedSlider);
    mainL->addLayout(controls);

    scene = new QGraphicsScene(this);
    view = new GraphView();
    view->setScene(scene);
    view->setRenderHint(QPainter::Antialiasing);
    view->setMinimumSize(1000, 800);
    view->setViewportUpdateMode(QGraphicsView::SmartViewportUpdate); // Redraw only things that actually change
    mainL->addWidget(view);

    connect(loadBtn, &QPushButton::clicked, this, &MainWindow::onLoad);
    connect(startBtn, &QPushButton::clicked, this, &MainWindow::onStart);
    connect(pauseBtn, &QPushButton::clicked, this, &MainWindow::onPause);
    connect(resetBtn, &QPushButton::clicked, this, &MainWindow::onReset);
    connect(view, &GraphView::sceneClicked, this, &MainWindow::onSceneClicked);
    connect(speedSlider, &QSlider::valueChanged, this, &MainWindow::onSpeedChanged);

    timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, &MainWindow::onStep);
}

void MainWindow::onLoad() {
    QString f = QFileDialog::getOpenFileName(this, "Open .osm.pbf", QString(), "*.osm.pbf");
    if (f.isEmpty()) return;

    graph.clear();
    graph.reserve(100000, 200000);
    scene->clear();
    nodesItems.clear();

    load_osm_pbf(f, graph);
    vt.compute(graph);

    for (size_t i = 0; i < graph.nodes.size(); ++i) {
        QPointF p = vt.toScene(graph.nodes[i].x, graph.nodes[i].y);
        // Using 1x1 rects or ellipses is much faster than complex shapes
        auto* el = scene->addEllipse(p.x()-0.5, p.y()-0.5, 1, 1, Qt::NoPen, QBrush(Qt::gray));
        el->setAcceptedMouseButtons(Qt::NoButton);
        el->setAcceptHoverEvents(false);
        el->setZValue(1);
        nodesItems.push_back(el);
    }
    onReset();
}

int MainWindow::batchSize() const {
    int v = speedSlider->value();
    v = std::max(0, v);
    return v;
}

void MainWindow::onStep() {
    if (!running) return;

    int stepsThisTick = batchSize();
    
    for (int i = 0; i < stepsThisTick; ++i) {
        if (pq.empty()) {
            onPause();
            return;
        }

        auto [f_v, d_v, v] = pq.top();
        pq.pop();

        if (d_v > dist[v]) continue;
        if (finalized[v]) continue;

        finalized[v] = 1;
        colorNodeByDistance(v);

        if (end_idx && v == *end_idx) {
            reconstructPath();
            onPause();
            return;
        }


        for (int eidx : graph.adj[v]) {
            auto &e = graph.edges[eidx];
            int u = e.v_idx;
            double new_d_u = dist[v] + e.weight;

            if (new_d_u < dist[u]) {
                dist[u] = new_d_u;
                parent[u] = v;

                double h = dist_heuristic(graph, u, *end_idx);
                pq.push({new_d_u + h, new_d_u, u});

                if (!e.lineItem) {
                    QPointF p1 = vt.toScene(graph.nodes[v].x, graph.nodes[v].y);
                    QPointF p2 = vt.toScene(graph.nodes[u].x, graph.nodes[u].y);
                    e.lineItem = scene->addLine(QLineF(p1, p2), QPen(Qt::white, 0.5));
                    e.lineItem->setZValue(0);
                }
            }
        }
    }
}

// Helper to reset a node's visual appearance
void MainWindow::updateNodeVisual(int idx, QColor color, bool isBig) {
    if (idx < 0 || idx >= nodesItems.size()) return;
    
    nodesItems[idx]->setBrush(QBrush(color));
    double size = isBig ? 8.0 : 2.0;
    double offset = size / 2.0;
    QPointF p = vt.toScene(graph.nodes[idx].x, graph.nodes[idx].y);
    nodesItems[idx]->setRect(p.x() - offset, p.y() - offset, size, size);
    
    // Bring endpoints to the very front
    if (isBig) nodesItems[idx]->setZValue(3);
    else nodesItems[idx]->setZValue(1);
}

void MainWindow::onSpeedChanged(int /*v*/) {
    if (running && timer) {
        // Restarting the timer applies the new interval immediately
        timer->start(timerInterval());
    }
}

void MainWindow::onStart() { 
    if (!start_idx || !end_idx) return;
    if (!astar_initialized) initAStar();
    running = true; 
    timer->start(timerInterval()); 
}

void MainWindow::onPause() { running = false; timer->stop(); }

void MainWindow::onReset() {
    onPause(); // Stop timer
    
    // 1. Remove all algorithm edges from scene
    for (auto &e : graph.edges) {
        if (e.lineItem) {
            scene->removeItem(e.lineItem);
            delete e.lineItem; // Free memory
            e.lineItem = nullptr;
        }
    }

    // 2. Reset all nodes to original state
    for (size_t i = 0; i < nodesItems.size(); ++i) {
        updateNodeVisual(i, Qt::gray, false);
    }

    // 3. Clear data
    start_idx.reset();
    end_idx.reset();
    dist.clear();
    parent.clear();
    finalized.clear();
    while(!pq.empty()) pq.pop();
    astar_initialized = false;
}

int MainWindow::timerInterval() const {
    return std::max(1, 1000 - (speedSlider->value() - 1) * 995 / 99);
}

void MainWindow::onSceneClicked(const QPointF& pt) {
    const double R = 10.0; // Click radius
    int best = -1; 
    double bestd2 = R*R;

    for (size_t i = 0; i < graph.nodes.size(); ++i) {
        QPointF p = vt.toScene(graph.nodes[i].x, graph.nodes[i].y);
        double dx = p.x() - pt.x();
        double dy = p.y() - pt.y();
        double d2 = dx*dx + dy*dy;
        if (d2 < bestd2) { bestd2 = d2; best = (int)i; }
    }

    if (best != -1) {
        if (!start_idx.has_value()) {
            start_idx = best;
            updateNodeVisual(best, Qt::red, true);
        } else if (!end_idx.has_value()) {
            end_idx = best;
            updateNodeVisual(best, Qt::cyan, true);
        } else {
            // 1. Reset old end_idx to tiny and white
            updateNodeVisual(end_idx.value(), Qt::white, false);
            
            // 2. Set new end_idx
            end_idx = best;
            updateNodeVisual(best, Qt::cyan, true);
        }
    }
}

void MainWindow::initAStar() {
    int n = graph.nodes.size();
    dist.assign(n, std::numeric_limits<double>::infinity());
    parent.assign(n, -1);
    finalized.assign(n, 0);
    while(!pq.empty()) pq.pop();
    
    dist[*start_idx] = 0.0;
    double h0 = dist_heuristic(graph, *start_idx, *end_idx);
    pq.push({h0, 0, *start_idx});
    astar_initialized = true;
}

void MainWindow::colorNodeByDistance(int idx) {
    double t = std::min(1.0, dist[idx] / 5000.0);
    QColor c((1.0-t)*255, 0, t*255);
    if (idx == start_idx) c = Qt::red;
    if (idx == end_idx) c = Qt::blue;
    nodesItems[idx]->setBrush(QBrush(c));
}

void MainWindow::reconstructPath() {
    int cur = *end_idx;
    while (cur != -1 && parent[cur] != -1) {
        int p = parent[cur];
        for (int eidx : graph.adj[p]) {
            if (graph.edges[eidx].v_idx == cur && graph.edges[eidx].lineItem) {
                graph.edges[eidx].lineItem->setPen(QPen(Qt::yellow, 2.0));
                graph.edges[eidx].lineItem->setZValue(3);
                break;
            }
        }
        nodesItems[cur]->setBrush(QBrush(Qt::yellow));
        cur = p;
    }
}
