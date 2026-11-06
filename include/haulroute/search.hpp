#pragma once

#include <cstdint>
#include <vector>

#include "haulroute/graph.hpp"

namespace haulroute {

struct QueryResult {
    Weight distance = kInf;
    std::uint32_t settled = 0;   // nodes popped from the queue: the honest measure of work
    std::vector<NodeId> path;    // filled only when asked for
    bool reachable() const { return distance != kInf; }
};

// Per-node scratch space that "clears" in O(1).
//
// Instead of resetting a million-entry array before every query, each slot
// remembers which query last wrote it. Anything stamped with an older round
// reads as untouched. Queries end up costing what they visit, not the size
// of the graph.
template <typename T>
class Scratch {
public:
    Scratch(std::size_t n, T empty) : values_(n, empty), stamp_(n, 0), empty_(empty) {}
    void next_round() {
        if (++round_ == 0) {  // wrapped after 4 billion queries; reset for real
            std::fill(stamp_.begin(), stamp_.end(), 0);
            round_ = 1;
        }
    }
    T get(std::size_t i) const { return stamp_[i] == round_ ? values_[i] : empty_; }
    void set(std::size_t i, T v) { values_[i] = v; stamp_[i] = round_; }

private:
    std::vector<T> values_;
    std::vector<std::uint32_t> stamp_;
    std::uint32_t round_ = 1;
    T empty_;
};

class Dijkstra {
public:
    explicit Dijkstra(const Graph& g);
    QueryResult run(NodeId s, NodeId t, bool want_path = false);

private:
    const Graph& g_;
    Scratch<Weight> dist_;
    Scratch<NodeId> parent_;
};

// Searches from both ends at once and stops when the frontiers can no
// longer improve on the best meeting point. Roughly halves the work.
class BidirectionalDijkstra {
public:
    explicit BidirectionalDijkstra(const Graph& g);
    QueryResult run(NodeId s, NodeId t, bool want_path = false);

private:
    const Graph& fwd_;
    Graph bwd_;
    Scratch<Weight> dist_f_, dist_b_;
    Scratch<NodeId> parent_f_, parent_b_;
};

// Dijkstra steered toward the target by a straight-line lower bound.
//
// The bound has to be in the same units as the weights (travel time), so the
// constructor measures the fastest "seconds per km" anywhere in the graph and
// scales distances by that. No arc beats it, so the estimate never overshoots
// and the answer stays exact.
class AStar {
public:
    explicit AStar(const Graph& g);
    QueryResult run(NodeId s, NodeId t, bool want_path = false);
    double weight_per_km() const { return factor_; }

private:
    Weight estimate(NodeId v, NodeId t) const;
    const Graph& g_;
    double factor_ = 0.0;
    Scratch<Weight> dist_;
    Scratch<NodeId> parent_;
};

std::vector<NodeId> walk_back(NodeId from, const Scratch<NodeId>& parent);

}  // namespace haulroute
