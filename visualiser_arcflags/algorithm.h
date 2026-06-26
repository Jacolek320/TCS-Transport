#ifndef ALGORITHM_H
#define ALGORITHM_H

#include <vector>
#include <queue>
#include "graph.h"

class ArcFlags {
    int start_idx, end_idx;
    std::vector<double> dist;
    std::vector<int> parent;
    std::vector<bool> finalized;
    std::priority_queue<std::pair<double,int>, std::vector<std::pair<double,int>>, std::greater<>> pq;

    std::vector<int> regions;           //what is the region of a given node, by idx
    std::vector<long long> flags;           //flags of a given edge, by eidx
public:
    Graph graph;
    ArcFlags();
    void set(int start, int end);
    void reset();
    double getDist(int idx);
    bool isFinished();
    
    void preprocess();
    std::vector<int> doSteps(int count);
    std::vector<int> reconstruct();
};

#endif