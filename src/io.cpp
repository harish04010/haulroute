#include "haulroute/io.hpp"

#include <cmath>
#include <fstream>
#include <random>
#include <sstream>
#include <stdexcept>

namespace haulroute {

namespace {

std::ifstream open_or_throw(const std::string& path) {
    std::ifstream in(path);
    if (!in) throw std::runtime_error("can't open " + path);
    return in;
}

}  // namespace

Graph load_dimacs(const std::string& gr_path, const std::string& co_path) {
    auto in = open_or_throw(gr_path);
    std::vector<Edge> edges;
    NodeId n = 0;
    std::string line;
    while (std::getline(in, line)) {
        if (line.empty() || line[0] == 'c') continue;
        std::istringstream ss(line);
        char kind;
        ss >> kind;
        if (kind == 'p') {
            std::string sp;
            std::size_t m;
            ss >> sp >> n >> m;
            edges.reserve(m);
        } else if (kind == 'a') {
            NodeId u, v;
            Weight w;
            ss >> u >> v >> w;
            edges.push_back({u - 1, v - 1, w});  // DIMACS is 1-indexed
        }
    }
    if (n == 0) throw std::runtime_error(gr_path + " has no problem line");
    Graph g = Graph::from_edges(n, edges);

    if (!co_path.empty()) {
        auto co = open_or_throw(co_path);
        std::vector<double> lon(n), lat(n);
        while (std::getline(co, line)) {
            if (line.empty() || line[0] != 'v') continue;
            std::istringstream ss(line);
            char kind;
            NodeId id;
            long long x, y;
            ss >> kind >> id >> x >> y;
            lon[id - 1] = static_cast<double>(x) / 1e6;  // microdegrees
            lat[id - 1] = static_cast<double>(y) / 1e6;
        }
        // Equirectangular projection to km. Plenty accurate at city scale,
        // and A* only needs distances that never overestimate.
        double mean_lat = 0;
        for (double v : lat) mean_lat += v;
        mean_lat /= n;
        const double kx = 111.32 * std::cos(mean_lat * M_PI / 180.0);
        g.x.resize(n);
        g.y.resize(n);
        for (NodeId i = 0; i < n; ++i) {
            g.x[i] = lon[i] * kx;
            g.y[i] = lat[i] * 110.57;
        }
    }
    return g;
}

Graph make_road_grid(std::uint32_t side, std::uint64_t seed) {
    std::mt19937_64 rng(seed);
    std::uniform_real_distribution<double> jitter(-0.3, 0.3);
    std::uniform_real_distribution<double> coin(0.0, 1.0);

    const NodeId n = side * side;
    std::vector<double> x(n), y(n);
    for (std::uint32_t r = 0; r < side; ++r) {
        for (std::uint32_t c = 0; c < side; ++c) {
            NodeId id = r * side + c;
            x[id] = c + jitter(rng);
            y[id] = r + jitter(rng);
        }
    }

    std::vector<Edge> edges;
    edges.reserve(static_cast<std::size_t>(n) * 4);
    auto connect = [&](NodeId a, NodeId b, double kmh) {
        double km = std::hypot(x[a] - x[b], y[a] - y[b]);
        auto w = static_cast<Weight>(std::lround(km / kmh * 36000.0));  // deciseconds
        if (w == 0) w = 1;
        edges.push_back({a, b, w});
        edges.push_back({b, a, w});
    };

    for (std::uint32_t r = 0; r < side; ++r) {
        for (std::uint32_t c = 0; c < side; ++c) {
            NodeId id = r * side + c;
            bool arterial_row = r % 8 == 0;
            bool arterial_col = c % 8 == 0;
            // Arterials always exist. Local streets are missing about 12% of the time,
            // which gives dead ends and detours instead of a perfect lattice.
            if (c + 1 < side && (arterial_row || coin(rng) > 0.12)) {
                connect(id, id + 1, arterial_row ? 80.0 : 40.0);
            }
            if (r + 1 < side && (arterial_col || coin(rng) > 0.12)) {
                connect(id, id + side, arterial_col ? 80.0 : 40.0);
            }
        }
    }

    Graph g = Graph::from_edges(n, edges);
    g.x = std::move(x);
    g.y = std::move(y);
    return g;
}

}  // namespace haulroute
