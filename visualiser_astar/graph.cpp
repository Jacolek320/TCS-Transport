#include "graph.h"
#include "utils.h"
#include <osmium/io/any_input.hpp>
#include <osmium/handler.hpp>
#include <osmium/visitor.hpp>
#include <osmium/geom/mercator_projection.hpp>
#include <osmium/io/header.hpp>
#include <iostream>

void Graph::reserve(size_t n_nodes, size_t n_edges) {
    nodes.reserve(n_nodes);
    edges.reserve(n_edges * 2);
    id_to_idx.reserve(n_nodes * 2);
    adj.reserve(n_nodes);
}

int Graph::add_node(const NodeId &id, double lon, double lat, double x, double y) {
    auto it = id_to_idx.find(id);
    if (it != id_to_idx.end()) return it->second;
    int idx = (int)nodes.size();
    nodes.push_back({id, lon, lat, x, y});
    id_to_idx[id] = idx;
    adj.emplace_back();
    return idx;
}

void Graph::add_edge(int u, int v, double w) {
    int e0 = (int)edges.size();
    edges.push_back({u, v, w, nullptr});
    adj[u].push_back(e0);
    edges.push_back({v, u, w, nullptr});
    adj[v].push_back(e0 + 1);
}

void Graph::clear() {
    nodes.clear();
    id_to_idx.clear();
    adj.clear();
    edges.clear();
}

void load_osm_pbf(const QString& path, Graph& graph) {
    osmium::io::File file(path.toStdString());
    osmium::io::Reader header_reader(file, osmium::osm_entity_bits::nothing);
    osmium::io::Header header = header_reader.header();
    header_reader.close();

    // 1. Get total bounds
    osmium::Box total_bbox = header.box();
    if (!total_bbox.valid()) {
        std::cerr << "File header has no bounding box. Filtering might fail." << std::endl;
        return; 
    }

    const double percentage = 1.0;
    // 0.0 -> load no points
    // 1.0 -> load whole map
    // anything in between -> load only according rectangle containing center

    // 2. Calculate the central window
    double center_lon = (total_bbox.bottom_left().lon() + total_bbox.top_right().lon()) / 2.0;
    double center_lat = (total_bbox.bottom_left().lat() + total_bbox.top_right().lat()) / 2.0;
    
    double half_w = (total_bbox.top_right().lon() - total_bbox.bottom_left().lon()) * percentage / 2.0;
    double half_h = (total_bbox.top_right().lat() - total_bbox.bottom_left().lat()) * percentage / 2.0;

    double min_lon = center_lon - half_w;
    double max_lon = center_lon + half_w;
    double min_lat = center_lat - half_h;
    double max_lat = center_lat + half_h;

    // 3. First pass: Only store node locations if they are inside the box
    std::unordered_map<NodeId, osmium::Location> node_locs;
    {
        osmium::io::Reader reader(file, osmium::osm_entity_bits::node);
        while (osmium::memory::Buffer buffer = reader.read()) {
            for (auto const& item : buffer) {
                if (item.type() != osmium::item_type::node) continue;
                const auto& n = static_cast<const osmium::Node&>(item);
                osmium::Location loc = n.location();
                
                // SPATIAL FILTER
                if (loc.lon() >= min_lon && loc.lon() <= max_lon &&
                    loc.lat() >= min_lat && loc.lat() <= max_lat) {
                    node_locs[n.id()] = loc;
                }
            }
        }
    }

    // 4. Second pass: Ways
    osmium::geom::MercatorProjection proj;
    {
        osmium::io::Reader reader(file, osmium::osm_entity_bits::way);
        while (osmium::memory::Buffer buffer = reader.read()) {
            for (auto const& item : buffer) {
                if (item.type() != osmium::item_type::way) continue;
                const auto& w = static_cast<const osmium::Way&>(item);
                if (!w.tags()["highway"]) continue;

                std::vector<int> idxs;
                for (const auto& nr : w.nodes()) {
                    auto it = node_locs.find(nr.ref());
                    // If node was filtered out in Pass 1, it won't be here
                    if (it == node_locs.end()) continue; 

                    osmium::Location loc = it->second;
                    auto coords = proj(loc);
                    int idx = graph.add_node(nr.ref(), loc.lon(), loc.lat(), coords.x, -coords.y);
                    idxs.push_back(idx);
                }

                // Add edges only between nodes that survived the filter
                for (size_t i = 1; i < idxs.size(); ++i) {
                    double wdist = haversine_m(graph.nodes[idxs[i-1]].lat, graph.nodes[idxs[i-1]].lon,
                                               graph.nodes[idxs[i]].lat, graph.nodes[idxs[i]].lon);
                    graph.add_edge(idxs[i-1], idxs[i], wdist);
                }
            }
        }
    }
}