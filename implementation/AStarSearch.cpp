#include "TopSpinStateSpace.h"
#include <iostream>
#include <cassert>
#include <queue>
#include <algorithm>
#include <vector>
#include <string>
#include <climits>
#include <unordered_map>
#include <chrono>
#include <numeric>
#include <random>
#include <functional>
#include <ranges>
#include <memory>
#include <cstdint>

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

using namespace std;

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

class AStarSearch {
private:
    long expandedNodes = 0;

    struct Node {
        TopSpinStateSpace::TopSpinState state;
        TopSpinStateSpace::TopSpinAction action;
        Node* parent;
        int cost;
        int h;

        Node(const TopSpinStateSpace::TopSpinState& s, Node* p,
             const TopSpinStateSpace::TopSpinAction& a, int c, int h)
            : state(s), parent(p), action(a), cost(c), h(h) {}
    };

    struct CompareNodes {
        bool operator()(const Node* a, const Node* b) const {
            int f_a = a->cost + a->h;
            int f_b = b->cost + b->h;
            if (f_a != f_b) return f_a > f_b;
            return a->cost < b->cost;
        }
    };

    vector<TopSpinStateSpace::TopSpinActionStatePair> extract_path(Node* node) {
        vector<TopSpinStateSpace::TopSpinActionStatePair> path;
        while (node && node->parent) {
            path.push_back(TopSpinStateSpace::TopSpinActionStatePair(node->action, node->state));
            node = node->parent;
        }
        reverse(path.begin(), path.end());
        return path;
    }

public:
    TopSpinStateSpace stateSpace;

    AStarSearch(const TopSpinStateSpace::TopSpinState& initialState)
        : stateSpace(initialState.size, initialState) {}

    void runSearchAlgorithm(const string& heuristic) {
        using namespace std::chrono;

        auto timeStart = high_resolution_clock::now();
        vector<TopSpinStateSpace::TopSpinActionStatePair> solution = run_Algorithm(heuristic);
        auto timeEnd = high_resolution_clock::now();

        TopSpinStateSpace::TopSpinState initialState = stateSpace.getInitialState();
        //normalize(&initialState);
        int initial_h = stateSpace.h(initialState, heuristic);

        cout << "Initial State: " << initialState << "| h = " << initial_h << endl;

        double elapsedSeconds = duration<double>(timeEnd - timeStart).count();
        cout << elapsedSeconds << " seconds search time" << endl;
        cout << "Number of expanded nodes: " << expandedNodes << endl;

        if (solution.empty()) {
            cout << "No solution" << endl;
        } else {
            int totalCost = 0;
            // Optional: Print the solution path
            // Commented out for experimentation purposes
            cout << "Solution:" << endl;
            for (const auto& pair : solution) {
                cout << "State: " << pair.state << "| h = " << stateSpace.h(pair.state, heuristic) << endl;
                totalCost += pair.action.cost();
            }
            cout << "Solution length: " << solution.size() << endl;
            cout << "Solution cost: " << totalCost << endl;
        }
    }

    vector<TopSpinStateSpace::TopSpinActionStatePair> run_Algorithm(const string& heuristic) {
        priority_queue<Node*, vector<Node*>, CompareNodes> open;
        unordered_map<size_t, int> closed;

        TopSpinStateSpace::TopSpinState initialState = stateSpace.getInitialState();
        //normalize(&initialState);
        int initial_h = stateSpace.h(initialState, heuristic);

        if (initial_h == INT_MAX)
            return {};

        Node* root = new Node(initialState, nullptr, TopSpinStateSpace::TopSpinAction(-1), 0, initial_h);
        open.push(root);

        while (!open.empty()) {
            Node* current = open.top();
            open.pop();
            
            size_t stateHash = std::hash<TopSpinStateSpace::TopSpinState>()(current->state);
            if (closed.count(stateHash)) {
                delete current;
                continue;
            }
            closed[stateHash] = current->cost;

            if (stateSpace.is_Goal(current->state)) {
                vector<TopSpinStateSpace::TopSpinActionStatePair> path = extract_path(current);
                while (!open.empty()) {
                    delete open.top();
                    open.pop();
                }
                delete current;
                return path;
            }

            auto successors = stateSpace.successors(current->state);
            for (const auto& [action, succState] : successors) {
                TopSpinStateSpace::TopSpinState nextState = succState;
                //normalize(&nextState);
                int g = current->cost + action.cost();
                int h = stateSpace.h(nextState, heuristic);
                if (h == INT_MAX) continue;
                Node* successor = new Node(nextState, current, action, g, h);
                open.push(successor);
            }
            expandedNodes++;
        }
        return {};
    }
};

int main(int argc, char* argv[]) {
    if (argc < 5) {
        std::cerr << "Usage: " << argv[0] << " n k m\n";
        return 1;
    }

    int n = std::atoi(argv[1]);
    int k = std::atoi(argv[2]);
    int m = std::atoi(argv[3]);
    string heuristic = argv[4];
    TopSpinStateSpace::TopSpinState initialState = createRandomState(n, k, m);
    AStarSearch search(initialState);
    search.runSearchAlgorithm(heuristic); 
    return 0;
}