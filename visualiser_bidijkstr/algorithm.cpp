#include "algorithm.h"

#include <iostream>

BiDijkstra::BiDijkstra() {

}
void BiDijkstra::set(int start, int end) {
    start_idx = start;
    end_idx = end;

    int n = graph.nodes.size();
    finished = false;
    dist_forward.assign(n, std::numeric_limits<double>::infinity());
    dist_backward.assign(n, std::numeric_limits<double>::infinity());
    parent_forward.assign(n, -1);
    parent_backward.assign(n, -1);
    while(!pq_forward.empty()) pq_forward.pop();            //clear
    while(!pq_backward.empty()) pq_backward.pop();          //clear
    
    dist_forward[start_idx] = 0.0;
    dist_backward[end_idx] = 0.0;
    pq_forward.push({0.0, start_idx});
    pq_backward.push({0.0, end_idx});
    optimal = std::numeric_limits<double>::infinity();
    optimal_edge = {-1, -1};
}
void BiDijkstra::reset() {
    finished = false;
    dist_forward.clear();
    dist_backward.clear();
    parent_forward.clear();
    parent_backward.clear();
    while(!pq_forward.empty()) pq_forward.pop();            //clear
    while(!pq_backward.empty()) pq_backward.pop();          //clear
}
double BiDijkstra::getDist(int idx) {
    return std::min(dist_forward[idx], dist_backward[idx]);
}
bool BiDijkstra::isFinished() {
    return finished;
}

std::vector<int> BiDijkstra::doSteps(int count) {
    std::vector<int> result;
    if(isFinished())
        return result;
    for (int i = 0; i < count; ++i) {
        if (pq_forward.empty() && pq_backward.empty()) {
            return result;
        }

        auto [dist_f, f] = pq_forward.top();
        auto [dist_b, b] = pq_backward.top();

        if(dist_f + dist_b > optimal) {         //check for algorithm termination
            finished = true;
            return result;
        }

        if(dist_f < dist_b) {           //forward
            pq_forward.pop();
            if(dist_f > dist_forward[f]) {
                i--;
                continue;// Skip stale entries without doint a step
            }

            for(int eidx : graph.adj[f]) {
                auto &e = graph.edges[eidx];
                int next = e.v_idx;
                double distance = dist_f + e.weight;
                if(distance < dist_forward[next]) {
                    dist_forward[next] = distance;
                    parent_forward[next] = f;
                    pq_forward.push({dist_forward[next], next});

                    double newOpt = distance + dist_backward[next];
                    if(newOpt < optimal) {
                        optimal = newOpt;
                        optimal_edge = {f, next};
                    }

                    // Visual feedback: Return the edge
                    result.push_back(eidx);
                }
            }
        }
        else {          //backward
            //the very same code as above
            pq_backward.pop();
            if(dist_b > dist_backward[b]) {
                i--;
                continue;// Skip stale entries without doint a step
            }

            for(int eidx : graph.adj[b]) {
                auto &e = graph.edges[eidx];
                int next = e.v_idx;
                double distance = dist_f + e.weight;
                if(distance < dist_backward[next]) {
                    dist_backward[next] = distance;
                    parent_backward[next] = b;
                    pq_backward.push({dist_backward[next], next});

                    double newOpt = distance + dist_forward[next];
                    if(newOpt < optimal) {
                        optimal = newOpt;
                        optimal_edge = {next, b};
                    }

                    // Visual feedback: Return the edge
                    result.push_back(eidx);
                }
            }
        }
    }
    return result;
}
std::vector<int> BiDijkstra::reconstruct() {
    std::vector<int> result;

    int cur_f = optimal_edge.first;
    while(cur_f != -1 && parent_forward[cur_f] != -1) {
        int p = parent_forward[cur_f];
        for (int eidx : graph.adj[p]) {
            if (graph.edges[eidx].v_idx == cur_f) {
                result.push_back(eidx);
                break;
            }
        }
        cur_f = p;
    }

    int cur_b = optimal_edge.second;
    while(cur_b != -1 && parent_backward[cur_b] != -1) {
        int p = parent_backward[cur_b];
        for (int eidx : graph.adj[p]) {
            if (graph.edges[eidx].v_idx == cur_b) {
                result.push_back(eidx);
                break;
            }
        }
        cur_b = p;
    }

    return result;
}
