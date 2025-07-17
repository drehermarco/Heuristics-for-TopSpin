#include "TopSpinStateSpace.h"
#include <iostream>
#include <vector>
#include <string>
#include <climits>
#include <chrono>
#include <algorithm>
#include <unordered_set>
#include <random>
#include <numeric>
#include <list>
#include <unordered_map>

using namespace std;

namespace std {
    template <>
    struct hash<TopSpinStateSpace::TopSpinState> {
        size_t operator()(const TopSpinStateSpace::TopSpinState& state) const {
            size_t h = 0;
            for (int x : state.permutation)
                h ^= hash<int>()(x) + 0x9e3779b9 + (h << 6) + (h >> 2);
            return h;
        }
    };
}

TopSpinStateSpace::TopSpinState createRandomState(int size, int k, int m) {
    std::vector<uint8_t> permutation(size);
    std::iota(permutation.begin(), permutation.end(), 1);
    TopSpinStateSpace::TopSpinState state(permutation, k);

    std::random_device rd;
    std::mt19937 rng(rd());
    std::uniform_int_distribution<int> dist(0, size - 1);

    for (int i = 0; i < m; i++) {
        int pos = dist(rng);
        TopSpinStateSpace::TopSpinAction action(pos);
        action.apply(state);
    }
    return state;
}

// Last recently used cache should help with node expansion reduction
template<typename Key, typename Value>
class LRUCache {
public:
    using ListIt = typename std::list<std::pair<Key, Value>>::iterator;
    LRUCache(size_t cap) : capacity(cap) {}

    bool get(const Key& key, Value& value) {
        auto it = map.find(key);
        if (it == map.end()) return false;
        cache.splice(cache.begin(), cache, it->second);
        value = it->second->second;
        return true;
    }
    void put(const Key& key, const Value& value) {
        auto it = map.find(key);
        if (it != map.end()) {
            it->second->second = value;
            cache.splice(cache.begin(), cache, it->second);
        } else {
            cache.emplace_front(key, value);
            map[key] = cache.begin();
            if (cache.size() > capacity) {
                map.erase(cache.back().first);
                cache.pop_back();
            }
        }
    }
    void clear() {
        cache.clear();
        map.clear();
    }
private:
    size_t capacity;
    std::list<std::pair<Key, Value>> cache;
    std::unordered_map<Key, ListIt> map;
};

class IDAStarSearch {
public:
    TopSpinStateSpace stateSpace;
    long long nodesExpanded = 0;
    double nextBound = 0.0;

    IDAStarSearch(const TopSpinStateSpace::TopSpinState& initialState)
        : stateSpace(initialState.size, initialState), transTable(TABLE_CAPACITY) {}

    static constexpr size_t TABLE_CAPACITY = 5'000'000;
    LRUCache<TopSpinStateSpace::TopSpinState, double> transTable;

    double search(const TopSpinStateSpace::TopSpinState& state,
        const TopSpinStateSpace::TopSpinState& parent,
        double g, double bound,
        const string& heuristic,
        vector<TopSpinStateSpace::TopSpinActionStatePair>& path,
        const TopSpinStateSpace::TopSpinState& goal,
        bool& found)
    {
        nodesExpanded++;
        double h = static_cast<double>(stateSpace.h(state, heuristic));
        double f = g + h;

        double prevG;
        if (transTable.get(state, prevG)) {
            if (g >= prevG) return h;
        }
        transTable.put(state, g);

        if (f > bound) {
            updateNextBound(bound, f);
            return h;
        }
        if (stateSpace.is_Goal(state)) {
            found = true;
            return 0.0;
        }

        auto successors = stateSpace.successors(state);

        std::sort(successors.begin(), successors.end(), [&](const auto& a, const auto& b) {
            double fa = g + a.action.cost() + stateSpace.h(a.state, heuristic);
            double fb = g + b.action.cost() + stateSpace.h(b.state, heuristic);
            return fa < fb;
        });

        for (auto& pair : successors) {
            TopSpinStateSpace::TopSpinState nextState = pair.state;
            if (nextState == parent) continue;

            path.push_back({pair.action, nextState});
            double edgeCost = static_cast<double>(pair.action.cost());
            double childH = search(nextState, state, g + edgeCost, bound, heuristic, path, goal, found);
            if (found) return 0.0;
            path.pop_back();

            if (childH - edgeCost > h) {
                h = childH - edgeCost;
                if (g + h > bound) {
                    updateNextBound(bound, g + h);
                    return h;
                }
            }
        }
        return h;
    }

    void updateNextBound(double currBound, double fCost) {
        fCost = floor(fCost);
        if (nextBound <= currBound)
            nextBound = fCost;
        else if (fCost > currBound && fCost < nextBound)
            nextBound = fCost;
    }

    void runSearchAlgorithm(const string& heuristic) {
        using namespace std::chrono;

        TopSpinStateSpace::TopSpinState initial = stateSpace.getInitialState();
        double bound = static_cast<double>(stateSpace.h(initial, heuristic));
        nextBound = bound;

        if (bound == static_cast<double>(INT_MAX)) {
            cout << "No solution found!" << endl;
            return;
        }

        auto timeStart = high_resolution_clock::now();

        vector<TopSpinStateSpace::TopSpinActionStatePair> path;
        vector<TopSpinStateSpace::TopSpinActionStatePair> solution;

        int iteration = 0;
        bool found = false;
        while (!found) {
            transTable.clear();
            path.clear();
            nextBound = 0.0;
            path.push_back({TopSpinStateSpace::TopSpinAction(-1), initial});
            double temp = search(initial, initial, 0.0, bound, heuristic, path, initial, found);
            path.erase(path.begin());
            iteration++;
            if (found) {
                solution = path;
                break;
            }
            if (nextBound == 0.0 || nextBound == bound) {
                cout << "No solution found" << endl;
                return;
            }
            bound = nextBound;
        }

        auto timeEnd = high_resolution_clock::now();
        double elapsedSeconds = duration<double>(timeEnd - timeStart).count();
        cout << "Initial State: " << initial << "| h = " << stateSpace.h(initial, heuristic) << endl;
        cout << elapsedSeconds << " seconds search time" << endl;
        cout << "Nodes expanded: " << nodesExpanded << endl;

        if (solution.empty()) {
            cout << "No solution" << endl;
        } else {
            int totalCost = 0;
            for (const auto& pair : solution) {
                totalCost += pair.action.cost();
            }
            cout << "Solution length: " << solution.size() << endl;
            cout << "Total cost: " << totalCost << endl;
            cout << "-----------------------------------------" << endl;
        }
    }
};

int main(int argc, char* argv[]) {
    if (argc < 5) {
        std::cerr << "Usage: " << argv[0] << " n k m heuristic\n";
        return 1;
    }

    int n = std::atoi(argv[1]);
    int k = std::atoi(argv[2]);
    int m = std::atoi(argv[3]);
    string heuristic = argv[4];
    TopSpinStateSpace::TopSpinState initialState = createRandomState(n, k, m);
    IDAStarSearch search(initialState);
    search.runSearchAlgorithm(heuristic);
    return 0;
}
