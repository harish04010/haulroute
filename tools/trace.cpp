// haulroute_trace: run all four algorithms on one route and write, as JSON,
// the order in which each one settled nodes. The web page replays it.
//
//   haulroute_trace [--grid N] [--seed S] > site/trace.json
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "haulroute/ch.hpp"
#include "haulroute/io.hpp"
#include "haulroute/search.hpp"

using namespace haulroute;

namespace {

NodeId nearest(const Graph& g, double fx, double fy) {
    double maxx = 0, maxy = 0;
    for (NodeId v = 0; v < g.node_count(); ++v) { maxx = std::max(maxx, g.x[v]); maxy = std::max(maxy, g.y[v]); }
    NodeId best = 0;
    double bd = 1e300;
    for (NodeId v = 0; v < g.node_count(); ++v) {
        double dx = g.x[v] - fx * maxx, dy = g.y[v] - fy * maxy, d = dx * dx + dy * dy;
        if (d < bd) { bd = d; best = v; }
    }
    return best;
}

void list(const char* key, const std::vector<NodeId>& v, bool last = false) {
    std::printf("\"%s\":[", key);
    for (std::size_t i = 0; i < v.size(); ++i) std::printf(i ? ",%u" : "%u", v[i]);
    std::printf("]%s", last ? "" : ",");
}

template <typename Algo>
void run(const char* name, Algo& algo, NodeId s, NodeId t, bool last) {
    std::vector<NodeId> order;
    algo.trace = &order;
    QueryResult r = algo.run(s, t, true);
    algo.trace = nullptr;
    std::printf("\"%s\":{\"distance\":%u,\"settled\":%u,", name, r.distance, r.settled);
    list("order", order);
    list("path", r.path, true);
    std::printf("}%s", last ? "" : ",");
}

}  // namespace

int main(int argc, char** argv) {
    std::uint32_t side = 80;
    std::uint64_t seed = 7;
    for (int i = 1; i + 1 < argc; ++i) {
        if (!std::strcmp(argv[i], "--grid")) side = std::stoul(argv[++i]);
        else if (!std::strcmp(argv[i], "--seed")) seed = std::stoull(argv[++i]);
    }
    Graph g = make_road_grid(side, seed);
    auto ch = ContractionHierarchy::build(g);
    NodeId s = nearest(g, 0.08, 0.12), t = nearest(g, 0.92, 0.86);

    Dijkstra dij(g);
    BidirectionalDijkstra bi(g);
    AStar astar(g);
    CHQuery chq(ch);

    std::printf("{\"nodes\":%u,\"side\":%u,\"seed\":%llu,\"source\":%u,\"target\":%u,", g.node_count(), side,
                static_cast<unsigned long long>(seed), s, t);
    std::printf("\"x\":[");
    for (NodeId v = 0; v < g.node_count(); ++v) std::printf(v ? ",%.2f" : "%.2f", g.x[v]);
    std::printf("],\"y\":[");
    for (NodeId v = 0; v < g.node_count(); ++v) std::printf(v ? ",%.2f" : "%.2f", g.y[v]);
    std::printf("],\"runs\":{");
    run("dijkstra", dij, s, t, false);
    run("bidirectional", bi, s, t, false);
    run("astar", astar, s, t, false);
    run("ch", chq, s, t, true);
    std::printf("}}\n");
}
