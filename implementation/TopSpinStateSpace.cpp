#include "TopSpinStateSpace.h"
#include "Abstraction.h"
#include "Heuristics.h"
#include <algorithm>
#include <vector>
#include <cstdint>
#include <cmath>

TopSpinStateSpace::TopSpinState::TopSpinState() : size(0) {}
TopSpinStateSpace::TopSpinState::TopSpinState(const std::vector<uint8_t>& perm, const int k) {
    permutation = perm;
    size = perm.size();
    this->k = k;
}

bool TopSpinStateSpace::TopSpinState::operator==(const TopSpinState& other) const {
    return permutation == other.permutation;
}

std::ostream& operator<<(std::ostream& os, const TopSpinStateSpace::TopSpinState& state) {
    for (size_t i = 0; i < state.permutation.size(); ++i) {
        os << static_cast<int>(state.permutation[i]) << " ";
    }
    return os;
}

TopSpinStateSpace::TopSpinAction::TopSpinAction(int r) {
    rotate = r;
}

int TopSpinStateSpace::TopSpinAction::cost() const {
    return 1;
}

void TopSpinStateSpace::TopSpinAction::apply(TopSpinState& state) const {
    if (rotate < 0) return;
    int n = static_cast<int>(state.permutation.size());
    int k = state.k;

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

TopSpinStateSpace::TopSpinActionStatePair::TopSpinActionStatePair(const TopSpinAction& a, const TopSpinState& s)
    : action(a), state(s) {}

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
    using HeuristicFunc = int(*)(const std::vector<uint8_t>&, int);
    static const std::unordered_map<std::string, HeuristicFunc> heuristics = {
        {"gap", topspin::gapHeuristic},
        {"manhattan", topspin::circularManhattanHeuristic},
        {"twoGroup", [](const std::vector<uint8_t>& s, int k) { return topspin::groupHeuristic(s, k, 2); }},
        {"threeGroup", [](const std::vector<uint8_t>& s, int k) { return topspin::groupHeuristic(s, k, 3); }},
        {"fourGroup", [](const std::vector<uint8_t>& s, int k) { return topspin::groupHeuristic(s, k, 4); }},
        {"fiveGroup", [](const std::vector<uint8_t>& s, int k) { return topspin::groupHeuristic(s, k, 5); }},
        {"oddEven", [](const std::vector<uint8_t>& s, int k) { return topspin::modDistance(s, k, 2); }},
        {"threeDistance", [](const std::vector<uint8_t>& s, int k) { return topspin::modDistance(s, k, 3); }},
        {"fourDistance", [](const std::vector<uint8_t>& s, int k) { return topspin::modDistance(s, k, 4); }},
        {"twoGroupC", [](const std::vector<uint8_t>& s, int k) { return topspin::groupHeuristicC(s, k, 2); }},
        {"threeGroupC", [](const std::vector<uint8_t>& s, int k) { return topspin::groupHeuristicC(s, k, 3); }},
        {"fourGroupC", [](const std::vector<uint8_t>& s, int k) { return topspin::groupHeuristicC(s, k, 4); }},
        {"oddEvenC", [](const std::vector<uint8_t>& s, int k) { return topspin::modDistanceC(s, k, 2); }},
        {"threeDistanceC", [](const std::vector<uint8_t>& s, int k) { return topspin::modDistanceC(s, k, 3); }},
        {"fourDistanceC", [](const std::vector<uint8_t>& s, int k) { return topspin::modDistanceC(s, k, 4); }},
        {"breakpoint", topspin::breakPointHeuristic}
    };

    auto it = heuristics.find(heuristic);
    if (it != heuristics.end()) {
        return it->second(state.permutation, state.k);
    }
    return INT_MAX;
}
