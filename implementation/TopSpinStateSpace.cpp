#include "TopSpinStateSpace.h"
#include "Abstraction.h"
#include "Heuristics.h"
#include <algorithm>
#include <vector>
#include <cmath>

// TopSpinState
TopSpinStateSpace::TopSpinState::TopSpinState() : size(0) {}
TopSpinStateSpace::TopSpinState::TopSpinState(const std::vector<int>& perm) {
    permutation = perm;
    size = perm.size();
    k = 4; // Assuming k is fixed at 4 for the TopSpin puzzle
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

int TopSpinStateSpace::h(const TopSpinState& state) const {
    return topspin::gapHeuristic(state.permutation, state.k);
    //return topspin::abstractHeuristicTwoGroup(state.permutation, state.k);
    //return topspin::abstractHeuristicThreeGroup(state.permutation, state.k);
    //return topspin::abstractHeuristicFourGroup(state.permutation, state.k);
    //return topspin::abstractHeuristicOE(state.permutation, state.k);
    //return topspin::abstractHeuristicDistance3(state.permutation, state.k);
    //return topspin::abstractHeuristicDistance4(state.permutation, state.k);
}
