#include "haulroute/search.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <queue>

namespace haulroute {

namespace {

using Entry = std::pair<std::uint64_t, NodeId>;  // (key, node); 64-bit so dist + estimate can't overflow
using MinHeap = std::priority_queue<Entry, std::vector<Entry>, std::greater<>>;

}  // namespace

std::vector<NodeId> walk_back(NodeId from, const Scratch<NodeId>& parent) {
    std::vector<NodeId> path;
    for (NodeId v = from; v != kNoNode; v = parent.get(v)) path.push_back(v);
    std::reverse(path.begin(), path.end());
    return path;
}

// ---------------------------------------------------------------- Dijkstra

Dijkstra::Dijkstra(const Graph& g)
    : g_(g), dist_(g.node_count(), kInf), parent_(g.node_count(), kNoNode) {}

QueryResult Dijkstra::run(NodeId s, NodeId t, bool want_path) {
    dist_.next_round();
    parent_.next_round();
    QueryResult r;
    MinHeap pq;
    dist_.set(s, 0);
    pq.push({0, s});

    while (!pq.empty()) {
        auto [d, u] = pq.top();
        pq.pop();
        if (d > dist_.get(u)) continue;  // stale entry; we found a shorter way since
        ++r.settled;
        if (u == t) {
            r.distance = static_cast<Weight>(d);
            break;
        }
        for (const Arc& a : g_.out(u)) {
            std::uint64_t nd = d + a.weight;
            if (nd < dist_.get(a.to)) {
                dist_.set(a.to, static_cast<Weight>(nd));
                parent_.set(a.to, u);
                pq.push({nd, a.to});
            }
        }
    }
    if (want_path && r.reachable()) r.path = walk_back(t, parent_);
    return r;
}

// ------------------------------------------------- Bidirectional Dijkstra

BidirectionalDijkstra::BidirectionalDijkstra(const Graph& g)
    : fwd_(g),
      bwd_(g.reversed()),
      dist_f_(g.node_count(), kInf),
      dist_b_(g.node_count(), kInf),
      parent_f_(g.node_count(), kNoNode),
      parent_b_(g.node_count(), kNoNode) {}

QueryResult BidirectionalDijkstra::run(NodeId s, NodeId t, bool want_path) {
    dist_f_.next_round();
    dist_b_.next_round();
    parent_f_.next_round();
    parent_b_.next_round();
    QueryResult r;

    MinHeap pf, pb;
    dist_f_.set(s, 0);
    dist_b_.set(t, 0);
    pf.push({0, s});
    pb.push({0, t});
    std::uint64_t best = kInf;
    NodeId meet = kNoNode;

    auto step = [&](MinHeap& pq, const Graph& g, Scratch<Weight>& mine, Scratch<Weight>& theirs,
                    Scratch<NodeId>& parent) {
        auto [d, u] = pq.top();
        pq.pop();
        if (d > mine.get(u)) return;
        ++r.settled;
        for (const Arc& a : g.out(u)) {
            std::uint64_t nd = d + a.weight;
            if (nd < mine.get(a.to)) {
                mine.set(a.to, static_cast<Weight>(nd));
                parent.set(a.to, u);
                pq.push({nd, a.to});
            }
            Weight other = theirs.get(a.to);
            if (other != kInf && nd + other < best) {
                best = nd + other;
                meet = a.to;
            }
        }
    };

    if (s == t) {
        best = 0;
        meet = s;
    }
    // Stop once the two smallest keys together can't beat what we already have.
    while (!pf.empty() && !pb.empty() && pf.top().first + pb.top().first < best) {
        if (pf.top().first <= pb.top().first) {
            step(pf, fwd_, dist_f_, dist_b_, parent_f_);
        } else {
            step(pb, bwd_, dist_b_, dist_f_, parent_b_);
        }
    }

    if (best != kInf) {
        r.distance = static_cast<Weight>(best);
        if (want_path) {
            r.path = walk_back(meet, parent_f_);
            for (NodeId v = parent_b_.get(meet); v != kNoNode; v = parent_b_.get(v)) r.path.push_back(v);
        }
    }
    return r;
}

// ------------------------------------------------------------------- A*

AStar::AStar(const Graph& g)
    : g_(g), dist_(g.node_count(), kInf), parent_(g.node_count(), kNoNode) {
    if (!g.has_coordinates()) return;  // no coordinates, no estimate: behaves like Dijkstra
    double best = std::numeric_limits<double>::infinity();
    for (NodeId u = 0; u < g.node_count(); ++u) {
        for (const Arc& a : g.out(u)) {
            double km = std::hypot(g.x[u] - g.x[a.to], g.y[u] - g.y[a.to]);
            if (km > 1e-9) best = std::min(best, a.weight / km);
        }
    }
    factor_ = std::isfinite(best) ? best : 0.0;
}

Weight AStar::estimate(NodeId v, NodeId t) const {
    if (factor_ == 0.0) return 0;
    double km = std::hypot(g_.x[v] - g_.x[t], g_.y[v] - g_.y[t]);
    // Flooring keeps the estimate admissible and consistent with integer weights.
    // The tiny shave covers floating-point error in the factor itself.
    return static_cast<Weight>(std::floor(km * factor_ * (1.0 - 1e-9)));
}

QueryResult AStar::run(NodeId s, NodeId t, bool want_path) {
    dist_.next_round();
    parent_.next_round();
    QueryResult r;
    MinHeap pq;
    dist_.set(s, 0);
    pq.push({estimate(s, t), s});

    while (!pq.empty()) {
        auto [key, u] = pq.top();
        pq.pop();
        Weight du = dist_.get(u);
        if (key > static_cast<std::uint64_t>(du) + estimate(u, t)) continue;
        ++r.settled;
        if (u == t) {
            r.distance = du;
            break;
        }
        for (const Arc& a : g_.out(u)) {
            std::uint64_t nd = static_cast<std::uint64_t>(du) + a.weight;
            if (nd < dist_.get(a.to)) {
                dist_.set(a.to, static_cast<Weight>(nd));
                parent_.set(a.to, u);
                pq.push({nd + estimate(a.to, t), a.to});
            }
        }
    }
    if (want_path && r.reachable()) r.path = walk_back(t, parent_);
    return r;
}

}  // namespace haulroute
