#pragma once

#include <cstdint>
#include <string>

#include "haulroute/graph.hpp"

namespace haulroute {

// Reads the DIMACS shortest-path challenge format (the USA road networks).
// The .co file is optional; without it A* falls back to plain Dijkstra.
Graph load_dimacs(const std::string& gr_path, const std::string& co_path = "");

// A synthetic road network: a jittered grid of local streets with some
// blocks removed, plus faster arterial roads every few rows and columns.
// Weights are travel times in tenths of a second.
Graph make_road_grid(std::uint32_t side, std::uint64_t seed);

}  // namespace haulroute
