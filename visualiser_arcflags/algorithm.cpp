#include "algorithm.h"

ArcFlags::ArcFlags() {

}
void ArcFlags::set(int start, int end) {
    start_idx = start;
    end_idx = end;

    int n = graph.nodes.size();
    dist.assign(n, std::numeric_limits<double>::infinity());
    parent.assign(n, -1);
    finalized.assign(n, false);
    while(!pq.empty()) pq.pop();            //clear pq
    
    dist[start_idx] = 0.0;
    pq.push({0.0, start_idx});
}
void ArcFlags::reset() {
    start_idx = -1;
    end_idx = -1;
    dist.clear();
    parent.clear();
    finalized.clear();
    while(!pq.empty()) pq.pop();            //clear pq
}
double ArcFlags::getDist(int idx) {
    return dist[idx];
}
bool ArcFlags::isFinished() {
    return finalized[end_idx];
}
std::vector<int> ArcFlags::doSteps(int count) {
    std::vector<int> result;
    if(isFinished())
        return result;
    for (int i = 0; i < count; ++i) {
        if (pq.empty()) {
            // onPause();
            return result;
        }

        auto [dist_u, u] = pq.top();
        pq.pop();

        if (dist_u > dist[u])  {
            i--;
            continue; // Skip stale entries without doint a step
        }

        finalized[u] = true;
        // colorNodeByDistance(u);

        // Check if we reached the target
        if(isFinished()) {
        // if (end_idx && u == *end_idx) {
            // reconstructPath();
            // onPause();
            return result;
        }

        for (int eidx : graph.adj[u]) {
            auto &e = graph.edges[eidx];
            int v = e.v_idx;
            double nd = dist[u] + e.weight;

            if (nd < dist[v]) {
                dist[v] = nd;
                parent[v] = u;
                pq.push({dist[v], v});

                // Visual feedback: Return the edge
                result.push_back(eidx);

                // if (!e.lineItem) {
                //     QPointF p1 = vt.toScene(graph.nodes[u].x, graph.nodes[u].y);
                //     QPointF p2 = vt.toScene(graph.nodes[v].x, graph.nodes[v].y);
                //     e.lineItem = scene->addLine(QLineF(p1, p2), QPen(Qt::white, 0.5));
                //     e.lineItem->setZValue(0);
                // }
            }
        }
    }
    return result;
}
std::vector<int> ArcFlags::reconstruct() {
    std::vector<int> result;
    int cur = end_idx;
    while(cur != -1 && parent[cur] != -1) {
        int p = parent[cur];
        for (int eidx : graph.adj[p]) {
            if (graph.edges[eidx].v_idx == cur && graph.edges[eidx].lineItem) {
                result.push_back(eidx);
                // graph.edges[eidx].lineItem->setPen(QPen(Qt::yellow, 2.0));
                // graph.edges[eidx].lineItem->setZValue(3); // Bring to front
                break;
            }
        }
        // nodesItems[cur]->setBrush(QBrush(Qt::yellow));
        cur = p;
    }
    return result;
}
