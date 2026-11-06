// haulroute: benchmark and query shortest-path algorithms on road networks.
//
//   haulroute bench [--grid N | --gr FILE [--co FILE]] [--queries Q] [--seed S]
//   haulroute route  [--grid N | --gr FILE [--co FILE]] --from A --to B

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <random>
#include <string>
#include <vector>

#include "haulroute/ch.hpp"
#include "haulroute/io.hpp"
#include "haulroute/search.hpp"

using namespace haulroute;
using Clock = std::chrono::steady_clock;

namespace {

struct Args {
    std::string command = "bench";
    std::string gr, co;
    std::uint32_t grid = 250;
    std::uint32_t queries = 1000;
    std::uint64_t seed = 1;
    long from = -1, to = -1;
};

Args parse(int argc, char** argv) {
    Args a;
    if (argc > 1 && argv[1][0] != '-') a.command = argv[1];
    for (int i = 1; i < argc; ++i) {
        auto is = [&](const char* flag) { return std::strcmp(argv[i], flag) == 0 && i + 1 < argc; };
        if (is("--gr")) a.gr = argv[++i];
        else if (is("--co")) a.co = argv[++i];
        else if (is("--grid")) a.grid = std::stoul(argv[++i]);
        else if (is("--queries")) a.queries = std::stoul(argv[++i]);
        else if (is("--seed")) a.seed = std::stoull(argv[++i]);
        else if (is("--from")) a.from = std::stol(argv[++i]);
        else if (is("--to")) a.to = std::stol(argv[++i]);
    }
    return a;
}

double since(Clock::time_point t) { return std::chrono::duration<double>(Clock::now() - t).count(); }

template <typename Algo>
void measure(const char* name, Algo& algo, const std::vector<std::pair<NodeId, NodeId>>& qs,
             const std::vector<Weight>& truth, double baseline_us, double* out_us) {
    std::uint64_t settled = 0;
    std::size_t wrong = 0;
    auto t0 = Clock::now();
    for (std::size_t i = 0; i < qs.size(); ++i) {
        auto r = algo.run(qs[i].first, qs[i].second);
        settled += r.settled;
        wrong += r.distance != truth[i];
    }
    double us = since(t0) * 1e6 / qs.size();
    if (out_us) *out_us = us;
    std::printf("  %-24s %10.1f  %12.0f  %9.1fx  %s\n", name, us, double(settled) / qs.size(),
                baseline_us > 0 ? baseline_us / us : 1.0, wrong ? "MISMATCH" : "ok");
}

}  // namespace

int main(int argc, char** argv) {
    Args a = parse(argc, argv);

    auto t0 = Clock::now();
    Graph g = a.gr.empty() ? make_road_grid(a.grid, a.seed) : load_dimacs(a.gr, a.co);
    std::printf("Graph: %u nodes, %zu arcs (%s), loaded in %.2fs\n", g.node_count(), g.arc_count(),
                a.gr.empty() ? ("synthetic " + std::to_string(a.grid) + "x" + std::to_string(a.grid) + " grid").c_str()
                             : a.gr.c_str(),
                since(t0));

    ContractionHierarchy::BuildStats stats;
    auto ch = ContractionHierarchy::build(g, {}, &stats);
    std::printf("Contraction hierarchy: %.2fs, %zu shortcuts (+%.0f%% edges)\n\n", stats.seconds, stats.shortcuts,
                100.0 * stats.shortcuts / stats.original_arcs);

    Dijkstra dij(g);
    BidirectionalDijkstra bi(g);
    AStar astar(g);
    CHQuery chq(ch);

    if (a.command == "route") {
        if (a.from < 0 || a.to < 0) {
            std::fprintf(stderr, "route needs --from and --to (0-based node ids)\n");
            return 2;
        }
        auto r = chq.run(static_cast<NodeId>(a.from), static_cast<NodeId>(a.to), true);
        if (!r.reachable()) {
            std::printf("No route from %ld to %ld.\n", a.from, a.to);
            return 1;
        }
        std::printf("Travel time %.1f min over %zu road segments.\nNodes:", r.distance / 600.0, r.path.size() - 1);
        for (NodeId v : r.path) std::printf(" %u", v);
        std::printf("\n");
        return 0;
    }

    std::mt19937_64 rng(a.seed * 31 + 7);
    std::uniform_int_distribution<NodeId> pick(0, g.node_count() - 1);
    std::vector<std::pair<NodeId, NodeId>> qs;
    std::vector<Weight> truth;
    while (qs.size() < a.queries) {
        NodeId s = pick(rng), t = pick(rng);
        Weight d = dij.run(s, t).distance;
        if (d == kInf) continue;  // only benchmark routes that exist
        qs.emplace_back(s, t);
        truth.push_back(d);
    }

    std::printf("%u random queries, all checked against Dijkstra\n\n", a.queries);
    std::printf("  %-24s %10s  %12s  %10s\n", "algorithm", "us/query", "nodes settled", "speedup");
    double base = 0;
    measure("Dijkstra", dij, qs, truth, 0, &base);
    measure("Bidirectional Dijkstra", bi, qs, truth, base, nullptr);
    measure("A* (straight-line bound)", astar, qs, truth, base, nullptr);
    measure("Contraction hierarchies", chq, qs, truth, base, nullptr);
    return 0;
}
