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

namespace topspin {

static int denom = 0;

int circularManhattanHeuristic(const std::vector<uint8_t>& state, int k) {
    int n = static_cast<int>(state.size());
    int count = 0;

    for (int i = 0; i < n; ++i) {
        int goal_pos = state[i] - 1;
        int dist = std::min((i - goal_pos + n) % n, (goal_pos - i + n) % n);
        count += dist;
    }
    if (denom == 0) {
        for (int i = 0; i < k; i++) {
            denom += std::abs(i - (k - 1 - i));
        }
    }
    return static_cast<int>(std::floor(count / static_cast<double>(denom)));
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
    return static_cast<int>(std::floor(count / 2.0));
}

int groupHeuristic(const std::vector<uint8_t>& state, int k, int numGroups) {
    int n = static_cast<int>(state.size());
    int bound = n / numGroups;
    std::vector<int> h(numGroups, 0);

    for (int g = 0; g < numGroups; ++g) {
        auto predicate = [g, bound, numGroups](int x) {
            return x > g * bound && x <= (g + 1) * bound;
        };
        std::vector<uint8_t> abstraction = topspin::abstract_state(state, predicate);
        h[g] = topspin::getSolutionLength(abstraction, k);
    }
    return *std::max_element(h.begin(), h.end());
}

int modDistance(const std::vector<uint8_t>& state, int k, int mod) {
    std::vector<int> h(mod, 0);
    for (int m = 0; m < mod; ++m) {
        auto predicate = [m, mod](int x) { return x % mod == m; };
        std::vector<uint8_t> abstraction = topspin::abstract_state(state, predicate);
        h[m] = topspin::getSolutionLength(abstraction, k);
    }
    return *std::max_element(h.begin(), h.end());
}

int twoGroupC(const std::vector<uint8_t>& state, int k) {
    int bound = state.size() / 2;
        auto mapping = [bound](int x) {
        if (x <= bound) return 1;
        else return 2;
    };
    std::vector<uint8_t> abstraction = topspin::abstract_stateC(state, mapping);
    int h = topspin::getSolutionLengthC(abstraction, k, mapping);
    return h;
}

int threeGroupC(const std::vector<uint8_t>& state, int k) {
    int bound = state.size() / 3;
    auto mapping = [bound](int x) {
        if (x <= bound) return 1;
        else if (x > bound && x <= 2 * bound) return 2;
        else return 3;
    };
    std::vector<uint8_t> abstraction = topspin::abstract_stateC(state, mapping);
    int h = topspin::getSolutionLengthC(abstraction, k, mapping);
    return h;
}

int fourGroupC(const std::vector<uint8_t>& state, int k) {
    int bound = state.size() / 4;
    auto mapping = [bound](int x) {
        if (x <= bound) return 1;
        else if (x > bound && x <= 2 * bound) return 2;
        else if (x > 2 * bound && x <= 3 * bound) return 3;
        else return 4;
    };
    std::vector<uint8_t> abstraction = topspin::abstract_stateC(state, mapping);
    int h = topspin::getSolutionLengthC(abstraction, k, mapping);
    return h;
}

int oddEvenC(const std::vector<uint8_t>& state, int k) {
    auto mapping = [](int x) { return x % 2; };
    std::vector<uint8_t> abstraction = topspin::abstract_stateC(state, mapping);
    int h = topspin::getSolutionLengthC(abstraction, k, mapping);
    return h;
}

int threeDistanceC(const std::vector<uint8_t>& state, int k) {
    auto mapping = [](int x) { return x % 3; };
    std::vector<uint8_t> abstraction = topspin::abstract_stateC(state, mapping);
    int h = topspin::getSolutionLengthC(abstraction, k, mapping);
    return h;
}

int fourDistanceC(const std::vector<uint8_t>& state, int k) {
    auto mapping = [](int x) { return x % 4; };
    std::vector<uint8_t> abstraction = topspin::abstract_stateC(state, mapping);
    int h = topspin::getSolutionLengthC(abstraction, k, mapping);
    return h;
}

// Breakpoint heuristic
int breakPointHeuristic(const std::vector<uint8_t>& state, int k) {
    int n = static_cast<int>(state.size());
    std::vector<int> p(n + 2);
    p[0] = 0;
    for (int i = 0; i < n; ++i) p[i + 1] = state[i];
    p[n + 1] = n + 1;
    n = p.size();

    using Graph = std::unordered_map<int, std::unordered_map<std::string, std::unordered_set<int>>>;
    Graph graph;
    int black_edges = 0;

    for (int i = 0; i < n - 1; ++i) {
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

    for (int i = 0; i < n - 1; ++i) {
        int u = p[i], v = p[i + 1];
        if (std::abs(u - v) == 1) {
            graph[u]["gray"].erase(v);
            graph[v]["gray"].erase(u);
        }
    }

    int cycle_count = 0;
    for (auto& [u, edges_u] : graph) {
        auto& black = edges_u["black"];
        auto& gray = edges_u["gray"];
        for (auto it = black.begin(); it != black.end();) {
            int v = *it;
            if (std::abs(u - v) == 2) {
                int w = (u + v) / 2;
                auto& gray_v = graph[v]["gray"];
                if (gray.count(w) && gray_v.count(w)) {
                    black.erase(it++);
                    graph[v]["black"].erase(u);
                    gray.erase(w);
                    gray_v.erase(w);
                    cycle_count++;
                    continue;
                }
            }
            ++it;
        }
    }

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
        bool found = true;
        while (found) {
            found = false;
            for (auto& [u, _] : g) {
                std::vector<int> path = {u};
                if (findKCycleFrom(g, path, k, true, count)) {
                    found = true;
                    break;
                }
            }
        }
    };

    cleanUpGraph(graph);
    for (int len = 2; len <= 20; ++len) {
        findKCycles(graph, 2 * len, cycle_count);
        cleanUpGraph(graph);
    }
    return static_cast<int>(std::ceil((black_edges - cycle_count) / 2.0));
}

}  // namespace topspin
