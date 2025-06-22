#include "TopSpinStateSpace.h"
#include <cstdlib>
#include <iostream>
#include <vector>
#include <string>
#include <climits>
#include <chrono>
#include <algorithm>
#include <numeric>
#include <random>
#include <unordered_set>

using namespace std;

TopSpinStateSpace::TopSpinState createRandomState(int size, int k, int m) {
    std::vector<int> permutation(size);
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

namespace std {
    template <>
    struct hash<TopSpinStateSpace::TopSpinState> {
        std::size_t operator()(const TopSpinStateSpace::TopSpinState& s) const {
            std::size_t h = 0;
            for (int val : s.permutation) {
                h ^= std::hash<int>()(val) + 0x9e3779b9 + (h << 6) + (h >> 2);
            }
            return h;
        }
    };
}

class IDAStarSearch {
private:
    long nodesExpanded = 0;

    int search(const TopSpinStateSpace& stateSpace,
            const TopSpinStateSpace::TopSpinState& state,
            int g, int threshold, const string& heuristic,
            std::vector<TopSpinStateSpace::TopSpinActionStatePair>& path,
            std::unordered_set<TopSpinStateSpace::TopSpinState>& visited,
            std::vector<TopSpinStateSpace::TopSpinActionStatePair>& solution) {
        int h = stateSpace.h(state, heuristic);
        int f = g + h;
        if (f > threshold) return f;
        if (stateSpace.is_Goal(state)) {
            solution = path;
            return -1;
        }

        int minNext = INT_MAX;
        auto successors = stateSpace.successors(state);
        std::sort(successors.begin(), successors.end(), [&](const auto& a, const auto& b) {
            return stateSpace.h(a.state, heuristic) < stateSpace.h(b.state, heuristic);
        });

        for (auto pair : successors) {
            normalize(&pair.state);
            if (visited.find(pair.state) != visited.end()) continue;

            path.push_back(pair);
            visited.insert(pair.state);
            nodesExpanded++;
            int temp = search(stateSpace, pair.state, g + pair.action.cost(), threshold, heuristic, path, visited, solution);
            if (temp == -1) return -1;
            if (temp < minNext) minNext = temp;
            path.pop_back();
            visited.erase(pair.state);
        }
        return minNext;
    }


public:
    TopSpinStateSpace stateSpace;

    IDAStarSearch(const TopSpinStateSpace::TopSpinState& initialState)
        : stateSpace(initialState.size, initialState) {}

    void runSearchAlgorithm(const string& heuristic) {
        using namespace std::chrono;

        cout << "Starting IDA* search..." << endl;
        TopSpinStateSpace::TopSpinState initial = stateSpace.getInitialState();
        normalize(&initial); // Normalize the initial state
        int threshold = stateSpace.h(initial, heuristic);
        cout << "Initial state: " << initial << endl;
        cout << "Heuristic value of initial state: " << threshold << endl;

        if (threshold == INT_MAX) {
            cout << "No solution: Heuristic returned INT_MAX" << endl;
            return;
        }

        auto timeStart = high_resolution_clock::now();

        std::vector<TopSpinStateSpace::TopSpinActionStatePair> path;
        std::vector<TopSpinStateSpace::TopSpinActionStatePair> solution;

        int iteration = 0;
        while (true) {
            std::unordered_set<TopSpinStateSpace::TopSpinState> visited;
            visited.insert(initial);
            int temp = search(stateSpace, initial, 0, threshold, heuristic, path, visited, solution);
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
        cout << "Search finished in " << elapsedSeconds << " seconds" << endl;
        cout << "Iterations (threshold increases): " << iteration << endl;
        cout << "Nodes expanded: " << nodesExpanded << endl;

        if (solution.empty()) {
            cout << "No solution" << endl;
        } else {
            int totalCost = 0;
            cout << "Solution path:" << endl;
            for (const auto& pair : solution) {
                cout << "State: " << pair.state << " | h = " << stateSpace.h(pair.state, heuristic) << endl;
                totalCost += pair.action.cost();
            }
            cout << "Solution length: " << solution.size() << endl;
            cout << "Total cost: " << totalCost << endl;
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
    const string heuristic = argv[4];

    TopSpinStateSpace::TopSpinState initialState = createRandomState(n, k, m);
    //TopSpinStateSpace::TopSpinState initialState({1, 2, 12, 8, 7, 14, 15, 10, 9, 19, 6, 16, 13, 18, 17, 11, 20, 3, 4, 5}, k);

    IDAStarSearch search(initialState);
    search.runSearchAlgorithm(heuristic);
    return 0;
}
