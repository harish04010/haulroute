#include "haulroute/graph.hpp"

#include <stdexcept>

namespace haulroute {

Graph Graph::from_edges(NodeId node_count, const std::vector<Edge>& edges) {
    Graph g;
    g.offsets_.assign(static_cast<std::size_t>(node_count) + 1, 0);

    // Counting sort by source node: count, prefix-sum, place.
    for (const Edge& e : edges) {
        if (e.from >= node_count || e.to >= node_count) {
            throw std::out_of_range("edge refers to a node outside the graph");
        }
        ++g.offsets_[e.from + 1];
    }
    for (std::size_t i = 1; i < g.offsets_.size(); ++i) {
        g.offsets_[i] += g.offsets_[i - 1];
    }

    g.arcs_.resize(edges.size());
    std::vector<std::uint32_t> cursor(g.offsets_.begin(), g.offsets_.end() - 1);
    for (const Edge& e : edges) {
        g.arcs_[cursor[e.from]++] = Arc{e.to, e.weight};
    }
    return g;
}

std::vector<Edge> Graph::edges() const {
    std::vector<Edge> out_edges;
    out_edges.reserve(arcs_.size());
    for (NodeId u = 0; u < node_count(); ++u) {
        for (const Arc& a : out(u)) {
            out_edges.push_back({u, a.to, a.weight});
        }
    }
    return out_edges;
}

Graph Graph::reversed() const {
    std::vector<Edge> flipped;
    flipped.reserve(arcs_.size());
    for (NodeId u = 0; u < node_count(); ++u) {
        for (const Arc& a : out(u)) {
            flipped.push_back({a.to, u, a.weight});
        }
    }
    Graph r = from_edges(node_count(), flipped);
    r.x = x;
    r.y = y;
    return r;
}

}  // namespace haulroute
