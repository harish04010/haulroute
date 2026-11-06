#pragma once

#include <cstdint>
#include <limits>
#include <vector>

namespace haulroute {

using NodeId = std::uint32_t;
using Weight = std::uint32_t;

inline constexpr Weight kInf = std::numeric_limits<Weight>::max();
inline constexpr NodeId kNoNode = std::numeric_limits<NodeId>::max();

struct Edge {
    NodeId from;
    NodeId to;
    Weight weight;
};

struct Arc {
    NodeId to;
    Weight weight;
};

// A read-only directed graph in compressed sparse row form.
//
// All outgoing arcs of a node sit next to each other in one array, so
// scanning a node's neighbors is a single linear read. On road networks
// that's the difference between the cache helping you and fighting you.
class Graph {
public:
    struct Range {
        const Arc* first;
        const Arc* last;
        const Arc* begin() const { return first; }
        const Arc* end() const { return last; }
        std::size_t size() const { return static_cast<std::size_t>(last - first); }
    };

    Graph() = default;
    static Graph from_edges(NodeId node_count, const std::vector<Edge>& edges);

    NodeId node_count() const { return offsets_.empty() ? 0 : static_cast<NodeId>(offsets_.size() - 1); }
    std::size_t arc_count() const { return arcs_.size(); }

    Range out(NodeId u) const { return {arcs_.data() + offsets_[u], arcs_.data() + offsets_[u + 1]}; }

    // Same nodes, every arc flipped. Used by searches that run backward from the target.
    Graph reversed() const;

    std::vector<Edge> edges() const;

    // Optional planar coordinates (km). Only A* uses them.
    bool has_coordinates() const { return x.size() == node_count() && node_count() > 0; }
    std::vector<double> x;
    std::vector<double> y;

private:
    std::vector<std::uint32_t> offsets_;
    std::vector<Arc> arcs_;
};

}  // namespace haulroute
