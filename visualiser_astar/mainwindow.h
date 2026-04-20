#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QWidget>
#include <QGraphicsView>
#include <QGraphicsScene>
#include <QPushButton>
#include <QSlider>
#include <QTimer>
#include <optional>
#include <queue>
#include "graph.h"
#include "utils.h"

class GraphView : public QGraphicsView {
    Q_OBJECT
public:
    explicit GraphView(QWidget* parent = nullptr) : QGraphicsView(parent) {}
protected:
    void mousePressEvent(QMouseEvent* ev) override;
signals:
    void sceneClicked(const QPointF& pt);
};

class MainWindow : public QWidget {
    Q_OBJECT
public:
    MainWindow();

private slots:
    void onLoad();
    void onStart();
    void onPause();
    void onReset();
    void onStep();
    void onSceneClicked(const QPointF& pt);
    void onSpeedChanged(int value);

private:
    void initAStar();
    void colorNodeByDistance(int idx);
    // void colorEdgeByDistance(int eidx, double distance);
    void reconstructPath();
    void updateNodeVisual(int idx, QColor color, bool isBig);
    int timerInterval() const;
    int batchSize() const;

    Graph graph;
    ViewTransform vt;
    QGraphicsScene* scene;
    GraphView* view;
    QSlider* speedSlider;
    QTimer* timer;
    std::vector<QGraphicsEllipseItem*> nodesItems;

    std::optional<int> start_idx, end_idx;
    std::vector<double> dist;
    std::vector<int> parent;
    std::vector<char> finalized;
    std::priority_queue<std::tuple<double,double,int>, std::vector<std::tuple<double,double,int>>, std::greater<>> pq;
    bool running = false;
    bool astar_initialized = false;
};

#endif
