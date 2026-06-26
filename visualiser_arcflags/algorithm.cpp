#include "algorithm.h"

#include <bits/stdc++.h>            //TODO: remove before commit

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

//split the map into (step)x(step) grid regions
//at most 8, because flags are stored in a long long
const int side = 3;
void ArcFlags::preprocess() {
    int node_count = graph.nodes.size();
    int edge_count = graph.edges.size();
    std::cout << "[PREPROCESSING] start" << std::endl;
    std::cout << "[PREPROCESSING] node_count = " << node_count << ", edge_count = " << edge_count << std::endl;

    //step 1: get bounding box limits
    double max_x = graph.nodes[0].x, min_x = graph.nodes[0].x;
    double max_y = graph.nodes[0].y, min_y = graph.nodes[0].y;
    for(Node &nd : graph.nodes) {
        max_x = std::max(max_x, nd.x);
        min_x = std::min(min_x, nd.x);
        max_y = std::max(max_y, nd.y);
        min_y = std::min(min_y, nd.y);
    }
    std::cout << "[PREPROCESSING] step 1 done" << std::endl;

    //step 2: calculate grid lines
    double base_x = min_x, base_y = min_y;
    double step_x = (max_x - min_x)/side;
    double step_y = (max_y - min_y)/side;
    std::cout << "[PREPROCESSING] step 2 done" << std::endl;

    //step 3: assign regions to nodes
    regions.assign(node_count, 0);
    for(int i=0; i<node_count; i++) {
        double cur_x = graph.nodes[i].x;
        double cur_y = graph.nodes[i].y;
        int slot_x = (cur_x - base_x)/step_x;
        slot_x = std::min(slot_x, side-1);           //avoid rounding errors...
        int slot_y = (cur_y - base_y)/step_y;
        slot_y = std::min(slot_y, side-1);           //...for nodes on the edges

        regions[i] = slot_y * side + slot_x;
    }
    std::cout << "[PREPROCESSING] step 3 done" << std::endl;

    std::unordered_map<int, int> counts;
    for(auto a : regions)
        counts[a] += 1;
    
    for(auto [val, cnt] : counts) {
        std::cout << "[PREPROCESSING] value " << val << "=(" << val/side << "," << val%side << ")" << " with count " << cnt << std::endl;
    }

    std::cout << "[PREPROCESSING] finished" << std::endl;


}
std::vector<int> ArcFlags::doSteps(int count) {
    std::vector<int> result;
    if(isFinished())
        return result;
    for (int i = 0; i < count; ++i) {
        if (pq.empty()) {
            return result;
        }

        auto [dist_u, u] = pq.top();
        pq.pop();

        if (dist_u > dist[u])  {
            i--;
            continue; // Skip stale entries without doint a step
        }

        finalized[u] = true;

        // Check if we reached the target
        if(isFinished()) {
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
            if (graph.edges[eidx].v_idx == cur) {
                result.push_back(eidx);
                break;
            }
        }
        cur = p;
    }
    return result;
}
