#include "algorithm.h"

#include <iostream>

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
const int side = 5;
int setRegions(Graph &graph, std::vector<int> &regions) {
    int node_count = graph.nodes.size();
    std::cout << "[SPLITTING] start" << std::endl;
    std::cout << "[SPLITTING] node_count = " << node_count << ", edge_count = " << graph.edges.size() << std::endl;

    //step 1: get bounding box limits
    double max_x = graph.nodes[0].x, min_x = graph.nodes[0].x;
    double max_y = graph.nodes[0].y, min_y = graph.nodes[0].y;
    for(Node &nd : graph.nodes) {
        max_x = std::max(max_x, nd.x);
        min_x = std::min(min_x, nd.x);
        max_y = std::max(max_y, nd.y);
        min_y = std::min(min_y, nd.y);
    }

    //step 2: calculate grid lines
    double base_x = min_x, base_y = min_y;
    double step_x = (max_x - min_x)/side;
    double step_y = (max_y - min_y)/side;

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

    std::unordered_map<int, int> counts;
    for(auto a : regions)
        counts[a] += 1;
    
    for(auto [val, cnt] : counts) {
        std::cout << "[SPLITTING] value " << val << "=(" << val/side << "," << val%side << ")" << " with count " << cnt << std::endl;
    }

    std::cout << "[SPLITTING] finished" << std::endl;
    return side*side;
}
void addFlags(int root_idx, int root_flag, Graph &graph, std::vector<std::vector<bool>> &flags) {
    //just dijkstra, mostly copied (or rewritten) from methods of ArcFlags class
    int node_count = graph.nodes.size();
    std::vector<double> dist(node_count, std::numeric_limits<double>::infinity());
    std::vector<int> parent(node_count, -1);
    std::vector<bool> finalized(node_count, false);
    std::priority_queue<std::pair<double,int>, std::vector<std::pair<double,int>>, std::greater<>> pq;
    dist[root_idx] = 0.0;
    pq.push({0.0, root_idx});

    while(!pq.empty()) {
        auto [dist_u, u] = pq.top();
        pq.pop();
        if(dist_u > dist[u])
            continue;
        finalized[u] = true;

        for(int eidx : graph.adj[u]) {
            auto &e = graph.edges[eidx];
            int v = e.v_idx;
            double new_distance = dist[u] + e.weight;
            
            if(new_distance < dist[v]) {
                dist[v] = new_distance;
                parent[v] = u;
                pq.push({dist[v], v});
            }
        }
    }

    //now: set flags in the reversed graph (towards the root_idx)
    //ie: if parent[a] = b, then the edge from `a` to `b` gets `root_flag`
    for(int u_idx = 0; u_idx < node_count; u_idx++) {
        if(parent[u_idx] == -1)
            continue;           //weird quirk
        for(int eidx : graph.adj[u_idx]) {          //a little brute force, but size is (near) constant, so it should be fine
            if(graph.edges[eidx].v_idx != parent[u_idx])
                continue;
            flags[eidx][root_flag] = true;
        }
    }
}
void ArcFlags::preprocess() {
    int region_count = setRegions(graph, regions);
    int edge_count = graph.edges.size();
    flags.assign(edge_count, std::vector<bool>(region_count, false));

    int pass_count = 0;
    for(auto &edge : graph.edges) {
        if(regions[edge.u_idx] != regions[edge.v_idx]) {
            pass_count += 1;
        }
    }
    std::cout << "[PREPROCESSING] of " << edge_count << " edges " << pass_count << " go across a region border" << std::endl;


    std::cout << "[FLAGGING] start" << std::endl;
    //set basic flags  and  find edges that cross region borders
    int counter = 0;
    for(int eidx=0; eidx < edge_count; eidx++) {
        int u = graph.edges[eidx].u_idx;
        int v = graph.edges[eidx].v_idx;
        flags[eidx][regions[v]] = true;     //each edge immediately gets a flag of it's target's region

        if(regions[u] == regions[v])
            continue;           //does not cross the edge
        addFlags(v, regions[v], graph, flags);
        counter++;
        if((100*counter)/pass_count > (100*(counter-1))/pass_count)
            std::cout << "[FLAGGING] "<<(100*counter)/pass_count << "%" << std::endl;
    }
    std::cout << "[FLAGGING] end" << std::endl;
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
            //skip if the edge does not have the flag
            if(!(flags[eidx][regions[end_idx]]))
                continue;

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
