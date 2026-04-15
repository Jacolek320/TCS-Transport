#ifndef GRAPH_H
#define GRAPH_H

#include <vector>
#include <unordered_map>
#include <cstdint>
#include <QGraphicsLineItem>

using NodeId = int64_t;

struct Node {
    NodeId id;
    double lon, lat;
    double x, y; // projected coordinates
};

struct Edge {
    int u_idx, v_idx;
    double weight;
    QGraphicsLineItem* lineItem = nullptr;
};

struct Graph {
    std::vector<Node> nodes;
    std::unordered_map<NodeId, int> id_to_idx;
    std::vector<std::vector<int>> adj; 
    std::vector<Edge> edges;

    void reserve(size_t n_nodes, size_t n_edges);
    int add_node(const NodeId &id, double lon, double lat, double x, double y);
    void add_edge(int u, int v, double w);
    void clear();
};

// Function declaration for loading
void load_osm_pbf(const QString& path, Graph& graph);

#endif