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
