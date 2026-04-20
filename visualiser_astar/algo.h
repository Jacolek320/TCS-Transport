#ifndef ALGO_H
#define ALGO_H

#include "graph.h"
#include <cmath>

// Euclidean distance as heuristic
inline double dist_heuristic(const Graph &graph, int u_id, int v_id) {
    const auto &a = graph.nodes[u_id];
    const auto &b = graph.nodes[v_id];
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    return std::sqrt(dx*dx + dy*dy);
}

#endif
