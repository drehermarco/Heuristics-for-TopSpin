#include "TopSpinStateSpace.h"
#include <algorithm>
#include <cmath>

// TopSpinState
TopSpinStateSpace::TopSpinState::TopSpinState() : size(0) {}
TopSpinStateSpace::TopSpinState::TopSpinState(const vector<int>& perm) {
    permutation = perm;
    size = perm.size();
}

bool TopSpinStateSpace::TopSpinState::operator==(const TopSpinState& other) const {
    return permutation == other.permutation;
}

ostream& operator<<(ostream& os, const TopSpinStateSpace::TopSpinState& state) {
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
    return 2; // Cost of the action is always 2 as each reverse can remove 2 gaps
}

void TopSpinStateSpace::TopSpinAction::apply(TopSpinState& state) const {
    if (rotate < 0) return;
    int n = state.size;
    swap(state.permutation[rotate % n], state.permutation[(rotate + 3) % n]);
    swap(state.permutation[(rotate + 1) % n], state.permutation[(rotate + 2) % n]);
}

ostream& operator<<(ostream& os, const TopSpinStateSpace::TopSpinAction& action) {
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

vector<TopSpinStateSpace::TopSpinActionStatePair> TopSpinStateSpace::successors(const TopSpinState& state) const {
    vector<TopSpinActionStatePair> result;
    for (const auto& action : actions) {
        TopSpinState newState = state;
        action.apply(newState);
        result.emplace_back(action, newState);
    }
    return result;
}

int TopSpinStateSpace::h(const TopSpinState& state) const {
    int count = 0;
    for (int i = 0; i < n; i++) {
        int current = state.permutation[i];
        int next = state.permutation[(i + 1) % n];
        if (current == n && next == 1) {
            continue; // Skip the wrap-around case for circular permutation
        }
        if (abs(current - next) > 1) {
            count++;
        }   
    }
    return count;
}
