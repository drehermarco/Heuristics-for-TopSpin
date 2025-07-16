#include "Heuristics.h"
#include "Abstraction.h"
#include <vector>
#include <functional>
#include <algorithm>
#include <iostream>
#include <cmath>
#include <unordered_map>
#include <unordered_set>
#include <cstdint>
#include <queue>
#include <climits>
#include <random>

namespace topspin {

static int denom = 0;

int circularManhattanHeuristic(const std::vector<uint8_t>& state, int k) {
    static int denom = 0;
    const int n = static_cast<int>(state.size());
    int best = INT_MAX;

    for (int rot = 0; rot < n; rot++) {
        int count = 0;
        for (int i = 0; i < n; i++) {
            int tile = state[(i + rot) % n];
            int goal_pos = tile - 1;
            int dist = std::min((i - goal_pos + n) % n, (goal_pos - i + n) % n);
            count += dist;
        }
        best = std::min(best, count);
    }

    if (denom == 0) {
        for (int i = 0; i < k; i++) {
            denom += std::abs(i - (k - 1 - i));
        }
    }

    return static_cast<int>(std::ceil(best / static_cast<double>(denom)));
}

int gapHeuristic(const std::vector<uint8_t>& state, int k) {
    int n = static_cast<int>(state.size());
    int count = 0;
    for (int i = 0; i < n; i++) {
        int current = state[i];
        int next = state[(i + 1) % n];
        if ((current == n && next == 1) || (current == 1 && next == n)) {
            continue; // Skip the wrap-around case for circular
        }
        if (abs(current - next) > 1) {
            count++;
        }   
    }
    return static_cast<int>(std::ceil(count / 2.0));
}

int groupHeuristic(const std::vector<uint8_t>& state, int k, int numGroups) {
    int n = static_cast<int>(state.size());
    int bound = (n + 1) / numGroups;
    std::vector<int> h(numGroups, 0);

    for (int g = 0; g < numGroups; g++) {
        auto predicate = [g, bound, numGroups, n](int x) {
            if (g == numGroups - 1) {
                return x > g * bound && x <= n;
            } else {
                return x > g * bound && x <= (g + 1) * bound;
            }
        };
        std::vector<uint8_t> abstraction = topspin::abstract_state(state, predicate);
        h[g] = topspin::getSolutionLength(abstraction, k);
    }
    return *std::max_element(h.begin(), h.end());
}

int modDistance(const std::vector<uint8_t>& state, int k, int mod) {
    std::vector<int> h(mod, 0);
    for (int m = 0; m < mod; m++) {
        auto predicate = [m, mod](int x) { return x % mod == m; };
        std::vector<uint8_t> abstraction = topspin::abstract_state(state, predicate);
        h[m] = topspin::getSolutionLength(abstraction, k);
    }
    return *std::max_element(h.begin(), h.end());
}

int groupHeuristicC(const std::vector<uint8_t>& state, int k, int numGroups) {
    int n = static_cast<int>(state.size());
    int bound = (n + 1) / numGroups;
    auto mapping = [bound, numGroups, n](int x) {
        int group = (x - 1) / bound + 1;
        if (group > numGroups) group = numGroups;
        return group;
    };
    std::vector<uint8_t> abstraction = topspin::abstract_stateC(state, mapping);
    int h = topspin::getSolutionLengthC(abstraction, k, mapping);
    return h;
}

int modDistanceC(const std::vector<uint8_t>& state, int k, int mod) {
    auto mapping = [mod](int x) { return x % mod; };
    std::vector<uint8_t> abstraction = topspin::abstract_stateC(state, mapping);
    int h = topspin::getSolutionLengthC(abstraction, k, mapping);
    return h;
}

// Breakpoint heuristic
struct pair_hash {
    template <class T1, class T2>
    std::size_t operator() (const std::pair<T1,T2>& p) const {
        auto h1 = std::hash<T1>{}(p.first);
        auto h2 = std::hash<T2>{}(p.second);
        return h1 ^ (h2 << 1);
    }
};

int breakpointCalculation(const std::vector<uint8_t>& state, int k) {
    int n = static_cast<int>(state.size());
    std::vector<int> p(n + 2);
    p[0] = 0;
    for (int i = 0; i < n; i++) p[i + 1] = state[i];
    p[n + 1] = n + 1;
    n = p.size();

    using Graph = std::unordered_map<int, std::unordered_map<std::string, std::unordered_set<int>>>;
    Graph graph;
    int black_edges = 0;

    for (int i = 0; i < n - 1; i++) {
        int u = p[i], v = p[i + 1];
        if (std::abs(u - v) != 1) {
            graph[u]["black"].insert(v);
            graph[v]["black"].insert(u);
            graph[u]["gray"].insert(u + 1);
            if (u != 0) graph[u]["gray"].insert(u - 1);
            if (v != n - 1) graph[v]["gray"].insert(v + 1);
            graph[v]["gray"].insert(v - 1);
            black_edges++;
        }
    }

    for (int i = 0; i < n - 1; i++) {
        int u = p[i], v = p[i + 1];
        if (std::abs(u - v) == 1) {
            graph[u]["gray"].erase(v);
            graph[v]["gray"].erase(u);
        }
    }
    int cycle_count = 0;
    auto cleanUpGraph = [](Graph& g) {
        std::vector<int> to_remove;
        for (auto& [node, edges] : g)
            if (edges["black"].empty() || edges["gray"].empty())
                to_remove.push_back(node);
        for (int node : to_remove) g.erase(node);
    };

    std::function<bool(Graph&, std::vector<int>&, int, bool, int&)> findKCycleFrom =
        [&](Graph& g, std::vector<int>& path, int k, bool use_black, int& count) {
            int current = path.back();
            if (path.size() == k) {
                if (!use_black && g[current]["gray"].count(path[0])) {
                    for (int i = 0; i < k; ++i) {
                        int u = path[i], v = path[(i + 1) % k];
                        std::string c = (i % 2 == 0) ? "black" : "gray";
                        g[u][c].erase(v);
                        g[v][c].erase(u);
                    }
                    count++;
                    return true;
                }
                return false;
            }
            const auto& edges = g[current][use_black ? "black" : "gray"];
            for (int neighbor : edges) {
                if (std::find(path.begin(), path.end(), neighbor) != path.end()) continue;
                path.push_back(neighbor);
                if (findKCycleFrom(g, path, k, !use_black, count)) return true;
                path.pop_back();
            }
            return false;
        };

    auto findKCycles = [&](Graph& g, int k, int& count) {
        std::vector<std::pair<int,int>> black_edges_list;

        std::unordered_set<std::pair<int,int>, pair_hash> seen;
        for (auto& [u, edges] : g) {
            for (int v : edges["black"]) {
                if (u < v) black_edges_list.emplace_back(u, v);
            }
        }

        std::random_device rd;
        std::mt19937 gen(rd());
        std::shuffle(black_edges_list.begin(), black_edges_list.end(), gen);

        for (auto& [u,v] : black_edges_list) {
            if (!g.count(u) || !g.count(v)) continue;

            std::vector<int> path = {u, v};
            int local_count = 0;

            if (findKCycleFrom(g, path, k, false, local_count)) {
                count += local_count;
            }
        }
    };

    for (int len = 2; len <= 10; len++) {
        findKCycles(graph, 2 * len, cycle_count);
        cleanUpGraph(graph);
    }
    return black_edges - cycle_count;
}

int breakpointHeuristic(const std::vector<uint8_t>& state, int k) {
    int n = static_cast<int>(state.size());
    // Initial state
    int best_h = INT_MAX;

    for (int i = 0; i < n/4; i++) {
        // Rotation with 1 at position 0
        auto it1 = std::find(state.begin(), state.end(), 1);
        if (it1 != state.end()) {
            int idx1 = static_cast<int>(std::distance(state.begin(), it1));
            std::vector<uint8_t> rot1(n);
            for (int i = 0; i < n; i++)
                rot1[i] = state[(i + idx1) % n];
            best_h = std::min(best_h, breakpointCalculation(rot1, k));
        }

        // Rotation with n at position n-1
        /*
        auto itn = std::find(state.begin(), state.end(), n);
        if (itn != state.end()) {
            int idxn = static_cast<int>(std::distance(state.begin(), itn));
            std::vector<uint8_t> rotn(n);
            for (int i = 0; i < n; i++)
                rotn[i] = state[(i + idxn - (n - 1) + n) % n];
            best_h = std::min(best_h, breakpointCalculation(rotn, k));
        }*/
    }
    return best_h;
}

}  // namespace topspin
