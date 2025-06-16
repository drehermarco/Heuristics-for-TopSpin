#include "TopSpinStateSpace.h"
#include "Abstraction.h"
#include "Heuristics.h"
#include <algorithm>
#include <vector>
#include <cmath>

// TopSpinState
TopSpinStateSpace::TopSpinState::TopSpinState() : size(0) {}
TopSpinStateSpace::TopSpinState::TopSpinState(const std::vector<int>& perm, const int k) {
    permutation = perm;
    size = perm.size();
    this->k = k;
}

bool TopSpinStateSpace::TopSpinState::operator==(const TopSpinState& other) const {
    return permutation == other.permutation;
}

std::ostream& operator<<(std::ostream& os, const TopSpinStateSpace::TopSpinState& state) {
    for (int i = 0; i < state.size; ++i) {
        os << state.permutation[i] << " ";
    }
    return os;
}

// TopSpinAction
TopSpinStateSpace::TopSpinAction::TopSpinAction(int r) {
    rotate = r;
}

int TopSpinStateSpace::TopSpinAction::cost() const {
    return 1;
}

void TopSpinStateSpace::TopSpinAction::apply(TopSpinState& state) const {
    if (rotate < 0) return;
    int n = static_cast<int>(state.permutation.size());
    int k = state.k;  // Make sure this value exists or is passed in appropriately

    for (int i = 0; i < k / 2; i++) {
        int left = (rotate + i) % n;
        int right = (rotate + k - 1 - i) % n;
        std::swap(state.permutation[left], state.permutation[right]);
    }
}

std::ostream& operator<<(std::ostream& os, const TopSpinStateSpace::TopSpinAction& action) {
    os << "Rotate at position: " << action.rotate;
    return os;
}

// TopSpinActionStatePair
TopSpinStateSpace::TopSpinActionStatePair::TopSpinActionStatePair(const TopSpinAction& a, const TopSpinState& s)
    : action(a), state(s) {}

// TopSpinStateSpace
TopSpinStateSpace::TopSpinStateSpace(int size, TopSpinState initial) {
    initialState = initial;
    n = initial.size;
    for (int i = 0; i < n; i++) {
        actions.push_back(TopSpinAction(i));
    }
}

TopSpinStateSpace::TopSpinState TopSpinStateSpace::getInitialState() const {
    return initialState;
}

bool TopSpinStateSpace::is_Goal(const TopSpinState& state) const {
    for (int i = 0; i < n; i++) {
        int a = state.permutation[i];
        int b = state.permutation[(i + 1) % n];
        if (a == n && b != 1) return false;
        if (a != n && a + 1 != b) return false;
    }
    return true;
}

std::vector<TopSpinStateSpace::TopSpinActionStatePair> TopSpinStateSpace::successors(const TopSpinState& state) const {
    std::vector<TopSpinActionStatePair> result;
    for (const auto& action : actions) {
        TopSpinState newState = state;
        action.apply(newState);
        result.emplace_back(action, newState);
    }
    return result;
}

int TopSpinStateSpace::h(const TopSpinState& state, const std::string& heuristic) const {
    using HeuristicFunc = int(*)(const std::vector<int>&, int);
    static const std::unordered_map<std::string, HeuristicFunc> heuristics = {
        {"gap", topspin::gapHeuristic},
        {"manhatten", topspin::circularManhattanHeuristic},
        {"twoGroup", topspin::twoGroup},
        {"threeGroup", topspin::threeGroup},
        {"fourGroup", topspin::fourGroup},
        {"oddEven", topspin::oddEven},
        {"threeDistance", topspin::threeDistance},
        {"fourDistance", topspin::fourDistance},
        {"twoGroupC", topspin::twoGroupC},
        {"threeGroupC", topspin::threeGroupC},
        {"fourGroupC", topspin::fourGroupC},
        {"oddEvenC", topspin::oddEvenC},
        {"threeDistanceC", topspin::threeDistanceC},
        {"fourDistanceC", topspin::fourDistanceC}
    };

    auto it = heuristics.find(heuristic);
    if (it != heuristics.end()) {
        return it->second(state.permutation, state.k);
    }
    return INT_MAX;
}
