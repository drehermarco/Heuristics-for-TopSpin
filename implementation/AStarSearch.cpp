#include "TopSpinStateSpace.h"
#include <cstdlib>
#include <iostream>
#include <cassert>
#include <queue>
#include <algorithm>
#include <vector>
#include <string>
#include <climits>
#include <unordered_map>
#include <chrono>
#include <memory>
#include <numeric>
#include <random>

namespace std {
    template <>
    struct hash<TopSpinStateSpace::TopSpinState> {
        size_t operator()(const TopSpinStateSpace::TopSpinState& state) const {
            return std::hash<std::string_view>()(
                std::string_view(reinterpret_cast<const char*>(state.permutation.data()),
                                 state.permutation.size() * sizeof(int)));
        }
    };
}

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

class NodePool {
    std::vector<std::unique_ptr<struct Node>> pool;
public:
    template<typename... Args>
    Node* allocate(Args&&... args) {
        pool.push_back(std::make_unique<Node>(std::forward<Args>(args)...));
        return pool.back().get();
    }
    void clear() { pool.clear(); }
};

struct Node {
    TopSpinStateSpace::TopSpinState state;
    Node* parent;
    TopSpinStateSpace::TopSpinAction action;
    int cost;
    int h;

    Node(const TopSpinStateSpace::TopSpinState& s, Node* p,
         const TopSpinStateSpace::TopSpinAction& a, int c, int h)
        : state(s), parent(p), action(a), cost(c), h(h) {}
};

struct CompareNodes {
    bool operator()(const Node* a, const Node* b) const {
        int f_a = a->cost + 1 * a->h;
        int f_b = b->cost + 1 * b->h;
        if (f_a != f_b) return f_a > f_b;
        return a->h > b->h;
    }
};

class AStarSearch {
private:
    long statesGenerated = 0;
    long expandedNodes = 0;

    std::vector<TopSpinStateSpace::TopSpinActionStatePair> extract_path(Node* node) {
        std::vector<TopSpinStateSpace::TopSpinActionStatePair> path;
        while (node && node->parent) {
            path.push_back({node->action, node->state});
            node = node->parent;
        }
        std::reverse(path.begin(), path.end());
        return path;
    }

public:
    TopSpinStateSpace stateSpace;

    AStarSearch(const TopSpinStateSpace::TopSpinState& initialState)
        : stateSpace(initialState.size, initialState) {}

    void runSearchAlgorithm(const string& heuristic) {
        using namespace std::chrono;

        cout << "Starting search..." << endl;
        auto timeStart = high_resolution_clock::now();
        vector<TopSpinStateSpace::TopSpinActionStatePair> solution = run_Algorithm(heuristic);
        auto timeEnd = high_resolution_clock::now();

        double elapsedSeconds = duration<double>(timeEnd - timeStart).count();
        cout << elapsedSeconds << " seconds search time" << endl;
        cout << "Number of expanded nodes: " << expandedNodes << endl;

        if (solution.empty()) {
            cout << "No solution" << endl;
        } else {
            int totalCost = 0;
            cout << "Solution:" << endl;
            for (const auto& pair : solution) {
                cout << "State: " << pair.state << " with heuristic: " << stateSpace.h(pair.state, heuristic) << endl;
                totalCost += pair.action.cost();
            }
            cout << "Solution length: " << solution.size() << endl;
            cout << "Solution cost: " << totalCost << endl;
        }
    }

    vector<TopSpinStateSpace::TopSpinActionStatePair> run_Algorithm(const string& heuristic) {
        NodePool nodePool;
        priority_queue<Node*, vector<Node*>, CompareNodes> open;
        unordered_map<TopSpinStateSpace::TopSpinState, int> g_values;

        auto initialState = stateSpace.getInitialState();
        int initial_h = stateSpace.h(initialState, heuristic);

        cout << "Initial state: " << initialState << endl;
        cout << "Heuristic value of initial state: " << initial_h << endl;

        if (initial_h == INT_MAX) return {};

        Node* root = nodePool.allocate(initialState, nullptr, TopSpinStateSpace::TopSpinAction(-1), 0, initial_h);
        open.push(root);
        g_values[initialState] = 0;
        statesGenerated++;

        while (!open.empty()) {
            Node* current = open.top();
            open.pop();

            if (stateSpace.is_Goal(current->state)) {
                auto path = extract_path(current);
                nodePool.clear();
                return path;
            }

            if (g_values[current->state] < current->cost) continue;

            for (const auto& pair : stateSpace.successors(current->state)) {
                int g = current->cost + pair.action.cost();
                int h = stateSpace.h(pair.state, heuristic);
                if (h == INT_MAX) continue;

                auto it = g_values.find(pair.state);
                if (it != g_values.end() && g >= it->second) continue;

                g_values[pair.state] = g;
                Node* child = nodePool.allocate(pair.state, current, pair.action, g, h);
                open.push(child);
                statesGenerated++;
            }
            expandedNodes++;
        }

        nodePool.clear();
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
    //TopSpinStateSpace::TopSpinState initialState({3, 10, 12, 11, 1, 7, 14, 15, 4, 8, 6, 16, 9, 5, 13, 2});
    AStarSearch search(initialState);
    search.runSearchAlgorithm(heuristic);
    return 0;
}