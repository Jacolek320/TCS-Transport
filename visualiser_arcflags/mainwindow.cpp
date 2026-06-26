#include "mainwindow.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFileDialog>
#include <QLabel>
#include <QMouseEvent>

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
    speedSlider->setRange(3, 18);
    speedSlider->setValue(6);

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

    std::cout << "Loading map (might take some time)" << std::endl;

    arcflags.graph.clear();
    arcflags.graph.reserve(100000, 200000);
    scene->clear();
    for(auto *it : nodesItems) {
        delete it;
    }
    nodesItems.clear();

    load_osm_pbf(f, arcflags.graph);
    vt.compute(arcflags.graph);
    arcflags.preprocess();

    for (size_t i = 0; i < arcflags.graph.nodes.size(); ++i) {
        QPointF p = vt.toScene(arcflags.graph.nodes[i].x, arcflags.graph.nodes[i].y);
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
    int v = (1<<speedSlider->value());
    v = std::max(0, v);
    return v;
}

void MainWindow::onStep() {
    if (!running) return;

    int stepsThisTick = batchSize();
    std::vector<int> usedNow = arcflags.doSteps(stepsThisTick);

    if(usedNow.empty()) {           //algorithm has failed
        onPause();
        return;
    }

    for(int eidx : usedNow) {
        auto &e = arcflags.graph.edges[eidx];
        int u = e.u_idx;
        colorNodeByDistance(u);
    }

    if(arcflags.isFinished()) {
        reconstructPath();
        onPause();
        return;
    }
}

// Helper to reset a node's visual appearance
void MainWindow::updateNodeVisual(int idx, QColor color, bool isBig) {
    if (idx < 0 || idx >= nodesItems.size()) return;
    
    nodesItems[idx]->setBrush(QBrush(color));
    double size = isBig ? 8.0 : 2.0;
    double offset = size / 2.0;
    QPointF p = vt.toScene(arcflags.graph.nodes[idx].x, arcflags.graph.nodes[idx].y);
    nodesItems[idx]->setRect(p.x() - offset, p.y() - offset, size, size);
    
    // Bring endpoints to the very front
    if (isBig) nodesItems[idx]->setZValue(4);
    else nodesItems[idx]->setZValue(2);
}

void MainWindow::onSpeedChanged(int /*v*/) {
    if (running && timer) {
        // Restarting the timer applies the new interval immediately
        timer->start(timerInterval());
    }
}

void MainWindow::onStart() {
    if (!start_idx || !end_idx) return;
    std::cout << "Started algorithm" << std::endl;
    if (!initialized) initAlg();
    running = true; 
    timer->start(timerInterval()); 
}

void MainWindow::onPause() {
    running = false;
    timer->stop();
}

void MainWindow::onReset() {
    std::cout << "Resetting map (might also take some time)" << std::endl;
    onPause(); // Stop timer
    
    // 1. Remove all algorithm edges from scene
    for(auto *line : edgesItems) {
        delete line;      //Free memory
    }
    edgesItems.clear();

    // 2. Reset all nodes to original state
    for (size_t i = 0; i < nodesItems.size(); ++i) {
        updateNodeVisual(i, Qt::gray, false);
    }

    // 3. Clear data
    arcflags.reset();
    start_idx.reset();
    end_idx.reset();
    initialized = false;
}

int MainWindow::timerInterval() const {
    return 0.02;//std::max(1, 1000 - (speedSlider->value() - 1) * 995 / 99);
}

void MainWindow::onSceneClicked(const QPointF& pt) {
    const double R = 10.0; // Click radius
    int best = -1; 
    double bestd2 = R*R;

    for (size_t i = 0; i < arcflags.graph.nodes.size(); ++i) {
        QPointF p = vt.toScene(arcflags.graph.nodes[i].x, arcflags.graph.nodes[i].y);
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

void MainWindow::initAlg() {
    arcflags.set(*start_idx, *end_idx);
    initialized = true;
}

void MainWindow::colorNodeByDistance(int idx) {
    double t = std::min(1.0, arcflags.getDist(idx) / 10000.0);
    QColor c((1.0-t)*255, 0, t*255);
    nodesItems[idx]->setBrush(QBrush(c));
}

void MainWindow::reconstructPath() {
    std::vector<int> pathFound = arcflags.reconstruct();

    for(int eidx : pathFound) {
        auto &e = arcflags.graph.edges[eidx];
        int u = e.u_idx;
        int v = e.v_idx;
        QPointF p1 = vt.toScene(arcflags.graph.nodes[u].x, arcflags.graph.nodes[u].y);
        QPointF p2 = vt.toScene(arcflags.graph.nodes[v].x, arcflags.graph.nodes[v].y);
        auto *line = scene->addLine(QLineF(p1, p2), QPen(Qt::yellow, 4.0));
        line->setZValue(3);       //Bring to front
        edgesItems.push_back(line);
        nodesItems[arcflags.graph.edges[eidx].v_idx]->setBrush(QBrush(Qt::yellow));
    }
}