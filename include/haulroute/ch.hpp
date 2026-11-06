#pragma once

#include <cstdint>
#include <vector>

#include "haulroute/graph.hpp"
#include "haulroute/search.hpp"

namespace haulroute {

// Contraction Hierarchies (Geisberger et al., 2008).
//
// Preprocessing removes nodes one at a time, least important first. Whenever
// removing a node would break a shortest path between two of its neighbors,
// a shortcut edge is added so the distance survives. Every node ends up with
// a rank, and any shortest path can be rebuilt as "go uphill in rank, then
// downhill". A query runs two small searches that only ever climb, and they
// meet at the top.
//
// The payoff: thousands of times fewer nodes touched per query on large road
// networks, at the cost of a one-time preprocessing pass.
class ContractionHierarchy {
public:
    struct Options {
        std::uint32_t witness_settle_limit = 500;  // cap on each witness search; lower = faster build, more shortcuts
    };

    struct BuildStats {
        double seconds = 0;
        std::size_t original_arcs = 0;
        std::size_t shortcuts = 0;
    };

    static ContractionHierarchy build(const Graph& g, Options opts, BuildStats* stats = nullptr);
    static ContractionHierarchy build(const Graph& g) { return build(g, Options{}, nullptr); }

    NodeId node_count() const { return static_cast<NodeId>(rank_.size()); }
    std::uint32_t rank(NodeId v) const { return rank_[v]; }

    // An edge in the search graph. `mid` is the node a shortcut skips over,
    // or kNoNode for an edge that exists in the original road network.
    struct UpEdge {
        NodeId to;
        Weight weight;
        NodeId mid;
    };

    struct UpRange {
        const UpEdge* first;
        const UpEdge* last;
        const UpEdge* begin() const { return first; }
        const UpEdge* end() const { return last; }
    };

    // Edges u -> v with rank(v) > rank(u), searched forward from the source.
    UpRange up_forward(NodeId u) const { return {fwd_.data() + fwd_off_[u], fwd_.data() + fwd_off_[u + 1]}; }
    // For an original edge u -> v with rank(u) > rank(v), stored at v pointing to u.
    // Searching these from the target climbs the hierarchy backward.
    UpRange up_backward(NodeId v) const { return {bwd_.data() + bwd_off_[v], bwd_.data() + bwd_off_[v + 1]}; }

    // Expands a CH edge u -> v back into the original road nodes, excluding u.
    void unpack(NodeId u, NodeId v, NodeId mid, std::vector<NodeId>& out) const;

private:
    const UpEdge* find_edge(NodeId u, NodeId v) const;

    std::vector<std::uint32_t> rank_;
    std::vector<std::uint32_t> fwd_off_, bwd_off_;
    std::vector<UpEdge> fwd_, bwd_;
};

// Query object: holds scratch space so many queries can reuse one allocation.
class CHQuery {
public:
    explicit CHQuery(const ContractionHierarchy& ch);
    QueryResult run(NodeId s, NodeId t, bool want_path = false);
    // When set, every settled node is appended in order. Off by default; used to visualize searches.
    std::vector<NodeId>* trace = nullptr;

private:
    const ContractionHierarchy& ch_;
    Scratch<Weight> dist_f_, dist_b_;
    Scratch<NodeId> parent_f_, parent_b_;
    Scratch<NodeId> mid_f_, mid_b_;
};

}  // namespace haulroute
