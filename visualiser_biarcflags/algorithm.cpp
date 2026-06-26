#include "algorithm.h"

#include <iostream>

ArcFlags::ArcFlags() {

}
void ArcFlags::set(int start, int end) {
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
    pq_forward.emplace(0.0, start_idx);
    pq_backward.emplace(0.0, end_idx);
    optimal = std::numeric_limits<double>::infinity();
    optimal_edge = {-1, -1};
}
void ArcFlags::reset() {
    finished = false;
    dist_forward.clear();
    dist_backward.clear();
    parent_forward.clear();
    parent_backward.clear();
    while(!pq_forward.empty()) pq_forward.pop();            //clear
    while(!pq_backward.empty()) pq_backward.pop();          //clear
}
double ArcFlags::getDist(int idx) {
    return std::min(dist_forward[idx], dist_backward[idx]);
}
bool ArcFlags::isFinished() {
    return finished;
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
    pq.emplace(0.0, root_idx);

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
                pq.emplace(dist[v], v);
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
        if (pq_forward.empty() || pq_backward.empty()) {
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
                //skip if the edge does not have the flag
                if(!(flags[eidx][regions[end_idx]]))
                    continue;

                auto &e = graph.edges[eidx];
                int next = e.v_idx;
                double distance = dist_f + e.weight;
                if(distance < dist_forward[next]) {
                    dist_forward[next] = distance;
                    parent_forward[next] = f;
                    pq_forward.emplace(dist_forward[next], next);

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
                //skip if the edge does not have the flag
                if(!(flags[eidx][regions[start_idx]]))
                    continue;

                auto &e = graph.edges[eidx];
                int next = e.v_idx;
                double distance = dist_f + e.weight;
                if(distance < dist_backward[next]) {
                    dist_backward[next] = distance;
                    parent_backward[next] = b;
                    pq_backward.emplace(dist_backward[next], next);

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
std::vector<int> ArcFlags::reconstruct() {
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
