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

void normalize(TopSpinStateSpace::TopSpinState* state) {
    auto it = std::find(state->permutation.begin(), state->permutation.end(), 1);
    if (it == state->permutation.end()) return;
    int idx = std::distance(state->permutation.begin(), it);
    std::rotate(state->permutation.begin(), state->permutation.begin() + idx, state->permutation.end());
}

class IDAStarSearch {
public:
    TopSpinStateSpace stateSpace;
    long long nodesExpanded = 0;

    IDAStarSearch(const TopSpinStateSpace::TopSpinState& initialState)
        : stateSpace(initialState.size, initialState) {}

    int search(const TopSpinStateSpace::TopSpinState& state,
           int g, int threshold,
           const string& heuristic,
           vector<TopSpinStateSpace::TopSpinActionStatePair>& path,
           unordered_set<TopSpinStateSpace::TopSpinState>& visited,
           vector<TopSpinStateSpace::TopSpinActionStatePair>& solution,
           unordered_map<TopSpinStateSpace::TopSpinState, int>& nodeTable)
    {
        int h = stateSpace.h(state, heuristic);
        int f = g + h;
        if (f > threshold) return f;
        if (stateSpace.is_Goal(state)) {
            solution = path;
            return -1;
        }

        // Node table check
        auto it = nodeTable.find(state);
        if (it != nodeTable.end() && g >= it->second) {
            return INT_MAX; // Already visited with lower or equal cost
        }
        nodeTable[state] = g;

        int minNext = INT_MAX;
        auto successors = stateSpace.successors(state);
        vector<pair<TopSpinStateSpace::TopSpinActionStatePair, int>> succWithH;
        succWithH.reserve(successors.size());

        for (auto& pair : successors) {
            TopSpinStateSpace::TopSpinState nextState = pair.state;
            if (visited.count(nextState)) continue;
            int hVal = stateSpace.h(nextState, heuristic);
            succWithH.push_back({{pair.action, nextState}, hVal});
        }

        std::sort(succWithH.begin(), succWithH.end(),
            [](auto& a, auto& b) {
                if (a.second != b.second)
                    return a.second < b.second;
                return a.first.action.cost() > b.first.action.cost();
            });

        for (auto& elem : succWithH) {
            auto& pair = elem.first;
            TopSpinStateSpace::TopSpinState nextState = pair.state;

            path.push_back({pair.action, nextState});
            visited.insert(nextState);
            nodesExpanded++;

            int temp = search(nextState, g + pair.action.cost(), threshold, heuristic, path, visited, solution, nodeTable);
            if (temp == -1) return -1;
            if (temp < minNext) minNext = temp;
            path.pop_back();
            visited.erase(nextState);
        }
        return minNext;
    }

    void runSearchAlgorithm(const string& heuristic) {
        using namespace std::chrono;

        TopSpinStateSpace::TopSpinState initial = stateSpace.getInitialState();
        int threshold = stateSpace.h(initial, heuristic);

        if (threshold == INT_MAX) {
            cout << "No solution found!" << endl;
            return;
        }

        auto timeStart = high_resolution_clock::now();

        vector<TopSpinStateSpace::TopSpinActionStatePair> path;
        vector<TopSpinStateSpace::TopSpinActionStatePair> solution;

        int iteration = 0;
        while (true) {
            unordered_set<TopSpinStateSpace::TopSpinState> visited;
            visited.insert(initial);
            unordered_map<TopSpinStateSpace::TopSpinState, int> nodeTable; // <-- add nodeTable
            int temp = search(initial, 0, threshold, heuristic, path, visited, solution, nodeTable);
            iteration++;
            if (temp == -1) break;
            if (temp == INT_MAX) {
                cout << "No solution found" << endl;
                return;
            }
            threshold = temp;
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
            //cout << "Solution path:" << endl;
            for (const auto& pair : solution) {
                //cout << "State: " << pair.state << " | h = " << stateSpace.h(pair.state, heuristic) << endl;
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