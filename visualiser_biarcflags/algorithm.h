#ifndef ALGORITHM_H
#define ALGORITHM_H

#include <vector>
#include <queue>
#include "graph.h"

class ArcFlags {
    int start_idx, end_idx;
    bool finished;
    std::vector<double> dist_forward;
    std::vector<double> dist_backward;
    std::vector<int> parent_forward;
    std::vector<int> parent_backward;
    std::priority_queue<std::pair<double,int>, std::vector<std::pair<double,int>>, std::greater<>> pq_forward;
    std::priority_queue<std::pair<double,int>, std::vector<std::pair<double,int>>, std::greater<>> pq_backward;
    double optimal;
    std::pair<int, int> optimal_edge;        //the former is towards start, the latter towards end
    
    std::vector<int> regions;           //what is the region of a given node, by idx
    std::vector<std::vector<bool>> flags;           //flags of a given edge, by eidx
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