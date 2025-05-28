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
#include <memory>

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

class AStarSearch {
private:
    long statesGenerated = 0;
    long expandedNodes = 0;

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
            int f_a = a->cost + 20 * a->h;
            int f_b = b->cost + 20 * b->h;

            if (f_a != f_b) {
                return f_a > f_b;
            } else {
                return a->h > b->h;
            }
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

    void runSearchAlgorithm() {
        using namespace std::chrono;

        cout << "Starting search..." << endl;
        auto timeStart = high_resolution_clock::now();
        vector<TopSpinStateSpace::TopSpinActionStatePair> solution = run_Algorithm();
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
                cout << "State: " << pair.state << " with heuristic: " << stateSpace.h(pair.state) << endl;
                totalCost += pair.action.cost();
            }
            cout << "Solution length: " << solution.size() << endl;
            cout << "Solution cost: " << totalCost << endl;
        }
    }

    vector<TopSpinStateSpace::TopSpinActionStatePair> run_Algorithm() {
        priority_queue<Node*, vector<Node*>, CompareNodes> open;
        unordered_map<TopSpinStateSpace::TopSpinState, int> closed;

        TopSpinStateSpace::TopSpinState initialState = stateSpace.getInitialState();
        int initial_h = stateSpace.h(initialState);

        cout << "Initial state: " << initialState << endl;
        cout << "Heuristic value of initial state: " << initial_h << endl;

        if (initial_h == INT_MAX)
            return {};

        Node* root = new Node(initialState, nullptr, TopSpinStateSpace::TopSpinAction(-1), 0, initial_h);
        open.push(root);
        statesGenerated++;

        while (!open.empty()) {
            Node* current = open.top();
            open.pop();

            auto closedIt = closed.find(current->state);
            if (closedIt != closed.end() && closedIt->second <= current->cost) {
                delete current;
                continue;
            }
            closed[current->state] = current->cost;

            if (stateSpace.is_Goal(current->state)) {
                vector<TopSpinStateSpace::TopSpinActionStatePair> path = extract_path(current);
                while (!open.empty()) {
                    delete open.top();
                    open.pop();
                }
                delete current;
                return path;
            }

            for (const auto& pair : stateSpace.successors(current->state)) {
                int g = current->cost + pair.action.cost();
                int h = stateSpace.h(pair.state);
                if (h == INT_MAX) continue;

                Node* successor = new Node(pair.state, current, pair.action, g, h);
                open.push(successor);
                statesGenerated++;
            }
            expandedNodes++;
        }

        return {};
    }
};

int main() {
    // Example usage
    TopSpinStateSpace::TopSpinState initialState({7, 1, 4, 9, 3, 6, 2, 5, 10, 8});
    //TopSpinStateSpace::TopSpinState initialState({3, 10, 12, 11, 1, 7, 4, 8, 6, 9, 5, 2});
    //TopSpinStateSpace::TopSpinState initialState({3, 10, 12, 11, 1, 7, 4, 13, 14, 8, 6, 9, 5, 2});
    //TopSpinStateSpace::TopSpinState initialState({3, 10, 12, 11, 1, 7, 14, 15, 4, 8, 6, 16, 9, 5, 13, 2});
    //TopSpinStateSpace::TopSpinState initialState({20, 19, 18, 17, 16, 15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1});
    //TopSpinStateSpace::TopSpinState initialState({4, 15, 8, 6, 1, 12, 11, 13, 5, 3, 2, 10, 16, 14, 7, 9});
    AStarSearch search(initialState);
    search.runSearchAlgorithm();
    return 0;
}
