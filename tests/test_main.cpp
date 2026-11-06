#include <cmath>
#include <cstdio>
#include <fstream>
#include <random>

#include "check.hpp"
#include "haulroute/ch.hpp"
#include "haulroute/io.hpp"
#include "haulroute/search.hpp"

using namespace haulroute;

namespace {

// Random sparse directed graph with coordinates. Weights are at least the
// straight-line distance, so it looks road-ish, but otherwise anything goes:
// one-way streets, parallel edges, self-loops, unreachable pockets.
Graph random_graph(NodeId n, std::size_t m, std::uint64_t seed) {
    std::mt19937_64 rng(seed);
    std::uniform_int_distribution<NodeId> pick(0, n - 1);
    std::uniform_real_distribution<double> coord(0, 50), slack(1.0, 3.0);
    std::vector<double> x(n), y(n);
    for (NodeId i = 0; i < n; ++i) {
        x[i] = coord(rng);
        y[i] = coord(rng);
    }
    std::vector<Edge> edges;
    for (std::size_t i = 0; i < m; ++i) {
        NodeId u = pick(rng), v = pick(rng);
        double km = std::hypot(x[u] - x[v], y[u] - y[v]);
        edges.push_back({u, v, static_cast<Weight>(km * 100 * slack(rng)) + 1});
    }
    Graph g = Graph::from_edges(n, edges);
    g.x = x;
    g.y = y;
    return g;
}

// A returned path must start and end in the right place, use real arcs,
// and add up to the reported distance.
bool path_is_valid(const Graph& g, const QueryResult& r, NodeId s, NodeId t) {
    if (r.path.empty() || r.path.front() != s || r.path.back() != t) return false;
    std::uint64_t total = 0;
    for (std::size_t i = 0; i + 1 < r.path.size(); ++i) {
        Weight best = kInf;
        for (const Arc& a : g.out(r.path[i])) {
            if (a.to == r.path[i + 1]) best = std::min(best, a.weight);
        }
        if (best == kInf) return false;
        total += best;
    }
    return total == r.distance;
}

}  // namespace

TEST(csr_layout_keeps_every_edge) {
    Graph g = Graph::from_edges(4, {{0, 1, 5}, {2, 3, 1}, {0, 2, 7}, {3, 0, 2}});
    EXPECT_EQ(g.node_count(), 4u);
    EXPECT_EQ(g.arc_count(), 4u);
    EXPECT_EQ(g.out(0).size(), 2u);
    EXPECT_EQ(g.out(1).size(), 0u);
    Graph r = g.reversed();
    EXPECT_EQ(r.out(0).size(), 1u);  // only 3 -> 0 points at node 0
}

TEST(scratch_clears_between_rounds) {
    Scratch<Weight> s(3, kInf);
    s.set(1, 42);
    EXPECT_EQ(s.get(1), 42u);
    s.next_round();
    EXPECT_EQ(s.get(1), kInf);
}

TEST(small_hand_checked_example) {
    //   0 --4--> 1 --1--> 3
    //   0 --1--> 2 --1--> 1
    Graph g = Graph::from_edges(4, {{0, 1, 4}, {1, 3, 1}, {0, 2, 1}, {2, 1, 1}});
    Dijkstra d(g);
    auto r = d.run(0, 3, true);
    EXPECT_EQ(r.distance, 3u);
    EXPECT(r.path == (std::vector<NodeId>{0, 2, 1, 3}));
    EXPECT(!d.run(3, 0).reachable());
    EXPECT_EQ(d.run(2, 2).distance, 0u);
}

TEST(all_algorithms_agree_on_random_graphs) {
    for (std::uint64_t seed = 1; seed <= 25; ++seed) {
        Graph g = random_graph(150, 600, seed);
        Dijkstra dij(g);
        BidirectionalDijkstra bi(g);
        AStar astar(g);
        auto ch = ContractionHierarchy::build(g);
        CHQuery chq(ch);

        std::mt19937_64 rng(seed * 7);
        std::uniform_int_distribution<NodeId> pick(0, g.node_count() - 1);
        for (int q = 0; q < 120; ++q) {
            NodeId s = pick(rng), t = pick(rng);
            Weight truth = dij.run(s, t).distance;
            EXPECT_EQ(bi.run(s, t).distance, truth);
            EXPECT_EQ(astar.run(s, t).distance, truth);
            EXPECT_EQ(chq.run(s, t).distance, truth);
        }
    }
}

TEST(paths_are_real_and_add_up) {
    Graph g = random_graph(300, 1400, 99);
    BidirectionalDijkstra bi(g);
    AStar astar(g);
    auto ch = ContractionHierarchy::build(g);
    CHQuery chq(ch);
    std::mt19937_64 rng(5);
    std::uniform_int_distribution<NodeId> pick(0, g.node_count() - 1);
    int checked = 0;
    for (int q = 0; q < 300; ++q) {
        NodeId s = pick(rng), t = pick(rng);
        auto r = chq.run(s, t, true);
        if (!r.reachable()) continue;
        ++checked;
        EXPECT(path_is_valid(g, r, s, t));
        EXPECT(path_is_valid(g, bi.run(s, t, true), s, t));
        EXPECT(path_is_valid(g, astar.run(s, t, true), s, t));
    }
    EXPECT(checked > 100);
}

TEST(road_grid_is_connected_enough_and_ch_matches) {
    Graph g = make_road_grid(40, 3);
    Dijkstra dij(g);
    auto ch = ContractionHierarchy::build(g);
    CHQuery chq(ch);
    std::mt19937_64 rng(11);
    std::uniform_int_distribution<NodeId> pick(0, g.node_count() - 1);
    int reachable = 0;
    for (int q = 0; q < 300; ++q) {
        NodeId s = pick(rng), t = pick(rng);
        auto truth = dij.run(s, t);
        reachable += truth.reachable();
        auto r = chq.run(s, t, true);
        EXPECT_EQ(r.distance, truth.distance);
        if (r.reachable()) EXPECT(path_is_valid(g, r, s, t));
    }
    EXPECT(reachable > 280);  // arterials keep the grid mostly connected
}

TEST(a_star_does_less_work_than_dijkstra_on_roads) {
    Graph g = make_road_grid(60, 8);
    Dijkstra dij(g);
    AStar astar(g);
    std::uint64_t dij_work = 0, astar_work = 0;
    std::mt19937_64 rng(2);
    std::uniform_int_distribution<NodeId> pick(0, g.node_count() - 1);
    for (int q = 0; q < 200; ++q) {
        NodeId s = pick(rng), t = pick(rng);
        dij_work += dij.run(s, t).settled;
        astar_work += astar.run(s, t).settled;
    }
    EXPECT(astar_work < dij_work);
}

TEST(dimacs_loader_reads_the_challenge_format) {
    const char* gr = "/tmp/haulroute_test.gr";
    const char* co = "/tmp/haulroute_test.co";
    {
        std::ofstream f(gr);
        f << "c tiny\np sp 3 3\na 1 2 10\na 2 3 5\na 1 3 20\n";
        std::ofstream c(co);
        c << "p aux sp co 3\nv 1 -73990000 40750000\nv 2 -73980000 40750000\nv 3 -73970000 40750000\n";
    }
    Graph g = load_dimacs(gr, co);
    EXPECT_EQ(g.node_count(), 3u);
    EXPECT_EQ(Dijkstra(g).run(0, 2).distance, 15u);
    EXPECT(g.has_coordinates());
    EXPECT(g.x[1] > g.x[0]);  // farther east
}

int main() { return check::run_all(); }
