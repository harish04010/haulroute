# Haulroute

A shortest-path engine for road networks in C++17, with four algorithms side by side: Dijkstra, bidirectional Dijkstra, A*, and Contraction Hierarchies. The same idea powers fast routing in map apps. Every answer is checked against plain Dijkstra, and the paths come back as real road segments, not just distances.

No dependencies beyond a C++17 compiler and CMake.

## Results

Synthetic road networks (see below), 1-core cloud VM, Release build. Every query's answer matches Dijkstra.

**62,500 intersections, 223k road segments, 1,000 queries**

| Algorithm | Time per query | Nodes settled | Speedup |
|---|---|---|---|
| Dijkstra | 4.10 ms | 30,658 | 1× |
| Bidirectional Dijkstra | 3.11 ms | 21,172 | 1.3× |
| A* | 2.11 ms | 9,608 | 1.9× |
| Contraction hierarchies | 0.062 ms | 309 | **66×** |

**250,000 intersections, 894k road segments, 500 queries**

| Algorithm | Time per query | Nodes settled | Speedup |
|---|---|---|---|
| Dijkstra | 20.9 ms | 127,341 | 1× |
| Bidirectional Dijkstra | 15.7 ms | 90,197 | 1.3× |
| A* | 9.55 ms | 40,235 | 2.2× |
| Contraction hierarchies | 0.173 ms | 639 | **121×** |

The interesting part is how the gap grows. Quadrupling the network makes Dijkstra 5.1× slower, while CH queries get 2.8× slower. A CH query climbs a hierarchy of increasingly important roads, and that hierarchy grows far more slowly than the map. The cost is preprocessing: 5.3 s and 23 s here, adding about 1.2 shortcuts per original edge. You pay it once per map, not once per query.

To run on a real network, `scripts/get_dimacs_ny.sh` downloads the New York City graph from the 9th DIMACS Challenge (about 264k nodes). Those results aren't in this README because I benchmarked on synthetic data.

## How each algorithm works

**Dijkstra** is the baseline: settle the closest unsettled node, relax its edges, repeat until you reach the target.

**Bidirectional Dijkstra** searches forward from the source and backward from the target over a reversed graph. It stops once the two smallest frontier keys together can't beat the best meeting point found so far. Two half-size circles cover less area than one big one.

**A\*** steers the search with a lower bound on the remaining travel time. The bound has to be in the same units as the edge weights (time, not distance), so the constructor scans every arc for the fastest seconds-per-km anywhere in the graph and multiplies straight-line distance by that. Because no road beats that pace, the estimate never overshoots and the answer stays exact. It's also floored to an integer, which keeps it consistent with integer weights.

**Contraction Hierarchies** removes nodes one at a time, least important first. When removing a node would break a shortest path between two of its neighbors, a shortcut edge preserves that distance. A bounded "witness search" checks whether a detour already exists first. If the search gives up early, it just adds a shortcut that might not be needed, so it can never produce a wrong answer. Node order uses edge difference plus a deleted-neighbors term, with lazy priority updates. A query runs two upward-only searches that meet at the top of the hierarchy. Shortcuts record the node they skip over, so a path unpacks recursively back into original road segments.

## Engineering details

- **CSR graph layout.** All outgoing arcs of a node sit next to each other in one array, built with a counting sort. Scanning neighbors is one linear read.
- **O(1) query reset.** The per-node distance arrays don't get cleared between queries. Each slot stores the round that last wrote it, and older stamps read as empty. On a 250k-node graph, a CH query touches about 640 nodes, so wiping 250k entries each time would cost more than the search itself.
- **Exactness, tested.** The tests run all four algorithms on 25 random graphs with one-way streets, parallel edges, self-loops, and unreachable pockets. They compare every distance to Dijkstra and check that every returned path uses real arcs and sums to the reported distance. They also run under AddressSanitizer and UBSan in CI, with both GCC and Clang.
- **64-bit keys.** Priority-queue keys are 64-bit, so distance plus estimate can't overflow on long routes.

## Synthetic networks

`make_road_grid(side, seed)` builds a jittered grid of 40 km/h local streets, with about 12% of blocks missing, and 80 km/h arterials every eighth row and column. Weights are travel times in tenths of a second. It's not a real city, but it has the property that matters: some roads matter far more than others. A uniform lattice would make CH look worse and A* look better than either really is.

## Build and run

```bash
cmake -S . -B build
cmake --build build -j
ctest --test-dir build --output-on-failure

./build/haulroute bench --grid 250 --queries 1000
./build/haulroute route --grid 60 --from 0 --to 3599

scripts/get_dimacs_ny.sh
./build/haulroute bench --gr data/USA-road-t.NY.gr --co data/USA-road-t.NY.co
```

Sanitizer build: `cmake -S . -B build-asan -DHAULROUTE_SANITIZE=ON -DCMAKE_BUILD_TYPE=Debug`.

## What I'd do next

- **Stall-on-demand** in CH queries, a standard trick that skips nodes reachable more cheaply from above. It typically cuts settled nodes by another 2 to 5×.
- **Faster preprocessing.** Contracting independent sets of nodes in parallel, and reusing witness searches, would bring the 23 s down a lot.
- **A binary format** for the preprocessed hierarchy, so it loads in milliseconds instead of being rebuilt.
- **Python bindings** with pybind11, so the analytics projects can call the router directly.
