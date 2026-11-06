#include "haulroute/ch.hpp"

#include <algorithm>
#include <chrono>
#include <functional>
#include <queue>
#include <stdexcept>

namespace haulroute {

namespace {

struct DynEdge {
    NodeId other;
    Weight weight;
    NodeId mid;
};

// The graph while it's being contracted: adjacency lists that can grow.
// Edges are never removed; contracted nodes are just skipped.
struct Workspace {
    std::vector<std::vector<DynEdge>> out, in;
    std::vector<bool> contracted;
    std::vector<std::uint32_t> deleted_neighbors;

    // Witness-search scratch
    Scratch<Weight> dist;
    explicit Workspace(NodeId n)
        : out(n), in(n), contracted(n, false), deleted_neighbors(n, 0), dist(n, kInf) {}

    // Insert u -> v, or lower its weight if a cheaper version shows up.
    void add_edge(NodeId u, NodeId v, Weight w, NodeId mid) {
        for (auto& e : out[u]) {
            if (e.other == v) {
                if (w < e.weight) {
                    e.weight = w;
                    e.mid = mid;
                    for (auto& r : in[v]) {
                        if (r.other == u) {
                            r.weight = w;
                            r.mid = mid;
                        }
                    }
                }
                return;
            }
        }
        out[u].push_back({v, w, mid});
        in[v].push_back({u, w, mid});
    }
};

struct Shortcut {
    NodeId from, to;
    Weight weight;
};

// Is there a path from `source` to every target that avoids `skip` and is no
// longer than going through it? Bounded Dijkstra; giving up early only means
// we add a shortcut we might not have needed, never a wrong answer.
void witness_search(Workspace& ws, NodeId source, NodeId skip, Weight limit, std::uint32_t settle_limit) {
    using Entry = std::pair<Weight, NodeId>;
    std::priority_queue<Entry, std::vector<Entry>, std::greater<>> pq;
    ws.dist.next_round();
    ws.dist.set(source, 0);
    pq.push({0, source});
    std::uint32_t settled = 0;
    while (!pq.empty()) {
        auto [d, u] = pq.top();
        pq.pop();
        if (d > ws.dist.get(u)) continue;
        if (d > limit || ++settled > settle_limit) break;
        for (const DynEdge& e : ws.out[u]) {
            if (e.other == skip || ws.contracted[e.other]) continue;
            Weight nd = d + e.weight;
            if (nd < ws.dist.get(e.other)) {
                ws.dist.set(e.other, nd);
                pq.push({nd, e.other});
            }
        }
    }
}

// Works out which shortcuts contracting v would need.
std::vector<Shortcut> shortcuts_for(Workspace& ws, NodeId v, std::uint32_t settle_limit) {
    std::vector<Shortcut> result;
    Weight max_out = 0;
    for (const DynEdge& o : ws.out[v]) {
        if (!ws.contracted[o.other]) max_out = std::max(max_out, o.weight);
    }
    for (const DynEdge& i : ws.in[v]) {
        NodeId u = i.other;
        if (ws.contracted[u]) continue;
        witness_search(ws, u, v, i.weight + max_out, settle_limit);
        for (const DynEdge& o : ws.out[v]) {
            NodeId x = o.other;
            if (ws.contracted[x] || x == u) continue;
            Weight via = i.weight + o.weight;
            if (ws.dist.get(x) > via) result.push_back({u, x, via});
        }
    }
    return result;
}

// Lower = contract sooner. Edge difference keeps the graph from filling up
// with shortcuts; the deleted-neighbors term spreads contraction evenly
// instead of eating one region at a time.
int priority(Workspace& ws, NodeId v, std::uint32_t settle_limit) {
    int degree = 0;
    for (const DynEdge& e : ws.in[v]) degree += !ws.contracted[e.other];
    for (const DynEdge& e : ws.out[v]) degree += !ws.contracted[e.other];
    int added = static_cast<int>(shortcuts_for(ws, v, settle_limit).size());
    return added - degree + static_cast<int>(ws.deleted_neighbors[v]);
}

}  // namespace

ContractionHierarchy ContractionHierarchy::build(const Graph& g, Options opts, BuildStats* stats) {
    auto start = std::chrono::steady_clock::now();
    const NodeId n = g.node_count();
    Workspace ws(n);
    for (NodeId u = 0; u < n; ++u) {
        for (const Arc& a : g.out(u)) {
            if (a.to != u) ws.add_edge(u, a.to, a.weight, kNoNode);  // drop self-loops, keep the cheapest parallel edge
        }
    }

    using Entry = std::pair<int, NodeId>;
    std::priority_queue<Entry, std::vector<Entry>, std::greater<>> queue;
    for (NodeId v = 0; v < n; ++v) queue.push({priority(ws, v, opts.witness_settle_limit), v});

    ContractionHierarchy ch;
    ch.rank_.assign(n, 0);
    std::size_t shortcut_count = 0;
    std::uint32_t next_rank = 0;

    while (!queue.empty()) {
        auto [p, v] = queue.top();
        queue.pop();
        if (ws.contracted[v]) continue;

        // Lazy update: priorities go stale as neighbors disappear. Recompute,
        // and if v is no longer the cheapest, put it back and try the next one.
        int fresh = priority(ws, v, opts.witness_settle_limit);
        if (!queue.empty() && fresh > queue.top().first) {
            queue.push({fresh, v});
            continue;
        }

        for (const Shortcut& s : shortcuts_for(ws, v, opts.witness_settle_limit)) {
            ws.add_edge(s.from, s.to, s.weight, v);
            ++shortcut_count;
        }
        ws.contracted[v] = true;
        ch.rank_[v] = next_rank++;
        for (const DynEdge& e : ws.in[v]) ++ws.deleted_neighbors[e.other];
        for (const DynEdge& e : ws.out[v]) ++ws.deleted_neighbors[e.other];
    }

    // Freeze into two upward CSR graphs.
    std::vector<std::vector<UpEdge>> fwd(n), bwd(n);
    for (NodeId u = 0; u < n; ++u) {
        for (const DynEdge& e : ws.out[u]) {
            if (ch.rank_[e.other] > ch.rank_[u]) {
                fwd[u].push_back({e.other, e.weight, e.mid});
            } else {
                bwd[e.other].push_back({u, e.weight, e.mid});
            }
        }
    }
    auto flatten = [n](const std::vector<std::vector<UpEdge>>& lists, std::vector<std::uint32_t>& off,
                       std::vector<UpEdge>& flat) {
        off.assign(static_cast<std::size_t>(n) + 1, 0);
        for (NodeId u = 0; u < n; ++u) off[u + 1] = off[u] + static_cast<std::uint32_t>(lists[u].size());
        flat.reserve(off[n]);
        for (const auto& l : lists) flat.insert(flat.end(), l.begin(), l.end());
    };
    flatten(fwd, ch.fwd_off_, ch.fwd_);
    flatten(bwd, ch.bwd_off_, ch.bwd_);

    if (stats) {
        stats->seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
        stats->original_arcs = g.arc_count();
        stats->shortcuts = shortcut_count;
    }
    return ch;
}

const ContractionHierarchy::UpEdge* ContractionHierarchy::find_edge(NodeId u, NodeId v) const {
    if (rank_[v] > rank_[u]) {
        for (const UpEdge& e : up_forward(u)) {
            if (e.to == v) return &e;
        }
    } else {
        for (const UpEdge& e : up_backward(v)) {
            if (e.to == u) return &e;
        }
    }
    return nullptr;
}

void ContractionHierarchy::unpack(NodeId u, NodeId v, NodeId mid, std::vector<NodeId>& out) const {
    if (mid == kNoNode) {
        out.push_back(v);
        return;
    }
    // A shortcut u -> v through mid is exactly the edges u -> mid and mid -> v,
    // each of which may itself be a shortcut.
    const UpEdge* first = find_edge(u, mid);
    const UpEdge* second = find_edge(mid, v);
    if (!first || !second) throw std::logic_error("shortcut points at a missing edge");
    unpack(u, mid, first->mid, out);
    unpack(mid, v, second->mid, out);
}

// ---------------------------------------------------------------- queries

CHQuery::CHQuery(const ContractionHierarchy& ch)
    : ch_(ch),
      dist_f_(ch.node_count(), kInf),
      dist_b_(ch.node_count(), kInf),
      parent_f_(ch.node_count(), kNoNode),
      parent_b_(ch.node_count(), kNoNode),
      mid_f_(ch.node_count(), kNoNode),
      mid_b_(ch.node_count(), kNoNode) {}

QueryResult CHQuery::run(NodeId s, NodeId t, bool want_path) {
    for (auto* sc : {&dist_f_, &dist_b_}) sc->next_round();
    for (auto* sc : {&parent_f_, &parent_b_, &mid_f_, &mid_b_}) sc->next_round();
    QueryResult r;

    using Entry = std::pair<std::uint64_t, NodeId>;
    std::priority_queue<Entry, std::vector<Entry>, std::greater<>> pf, pb;
    dist_f_.set(s, 0);
    dist_b_.set(t, 0);
    pf.push({0, s});
    pb.push({0, t});
    std::uint64_t best = kInf;
    NodeId meet = kNoNode;

    // Unlike plain bidirectional search, the two sides can't stop as soon as
    // their keys add up to `best`: the meeting point is the top of a hill,
    // not the middle of the route. Each side runs until its own smallest key
    // reaches `best`.
    auto step = [&](auto& pq, bool forward) {
        auto [d, u] = pq.top();
        pq.pop();
        auto& mine = forward ? dist_f_ : dist_b_;
        auto& theirs = forward ? dist_b_ : dist_f_;
        if (d > mine.get(u)) return;
        ++r.settled;
        Weight other = theirs.get(u);
        if (other != kInf && d + other < best) {
            best = d + other;
            meet = u;
        }
        auto edges = forward ? ch_.up_forward(u) : ch_.up_backward(u);
        for (const auto& e : edges) {
            std::uint64_t nd = d + e.weight;
            if (nd < mine.get(e.to)) {
                mine.set(e.to, static_cast<Weight>(nd));
                (forward ? parent_f_ : parent_b_).set(e.to, u);
                (forward ? mid_f_ : mid_b_).set(e.to, e.mid);
                pq.push({nd, e.to});
            }
        }
    };

    while (true) {
        bool f_live = !pf.empty() && pf.top().first < best;
        bool b_live = !pb.empty() && pb.top().first < best;
        if (!f_live && !b_live) break;
        if (f_live && (!b_live || pf.top().first <= pb.top().first)) {
            step(pf, true);
        } else {
            step(pb, false);
        }
    }

    if (best == kInf) return r;
    r.distance = static_cast<Weight>(best);

    if (want_path) {
        // Climb from s to the peak, then come down from the peak to t,
        // expanding every shortcut on the way.
        std::vector<NodeId> up;
        for (NodeId v = meet; v != s; v = parent_f_.get(v)) up.push_back(v);
        std::reverse(up.begin(), up.end());
        r.path.push_back(s);
        NodeId prev = s;
        for (NodeId v : up) {
            ch_.unpack(prev, v, mid_f_.get(v), r.path);
            prev = v;
        }
        for (NodeId v = meet; v != t;) {
            NodeId next = parent_b_.get(v);
            // The backward search reached v from `next`, so the edge v -> next
            // (in road direction) was recorded on v.
            ch_.unpack(v, next, mid_b_.get(v), r.path);
            v = next;
        }
    }
    return r;
}

}  // namespace haulroute
