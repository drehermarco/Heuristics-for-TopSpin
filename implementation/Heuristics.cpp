#include "Heuristics.h"
#include "Abstraction.h"
#include <vector>
#include <functional>
#include <algorithm>
#include <iostream>
#include <cmath>

namespace topspin {

int circularManhattanHeuristic(const std::vector<int>& state, int k) {
    auto it = std::ranges::find(state, 1);
    int current_index = std::distance(state.begin(), it);

    // Shift so '1' is at the front
    int shift = (0 - current_index + state.size()) % state.size();
    auto perm = state;
    std::rotate(perm.begin(), perm.begin() + (perm.size() - shift), perm.end());

    int n = static_cast<int>(perm.size());
    int count = 0;

    for (int i = 0; i < n; ++i) {
        int goal_pos = perm[i] - 1;
        int dist = std::min((i - goal_pos + n) % n, (goal_pos - i + n) % n);
        count += dist;
    }
    return static_cast<int>(std::round(count / static_cast<double>(k * (k - 1))));
}

int gapHeuristic(const std::vector<int>& state, int k) {
    int n = static_cast<int>(state.size());
    int count = 0;
    for (int i = 0; i < n; i++) {
        int current = state[i];
        int next = state[(i + 1) % n];
        if ((current == n && next == 1) || (current == 1 && next == n)) {
            continue; // Skip the wrap-around case for circular
        }
        if (abs(current - next) > 1) {
            count++;
        }   
    }
    return static_cast<int>(std::round(count / 2.0));
}

int groupHeuristic(const std::vector<int>& state, int k, int numGroups) {
    int n = static_cast<int>(state.size());
    int bound = n / numGroups;
    std::vector<int> h(numGroups, 0);

    for (int g = 0; g < numGroups; ++g) {
        auto predicate = [g, bound, numGroups](int x) {
            return x > g * bound && x <= (g + 1) * bound;
        };
        std::vector<int> abstraction = topspin::abstract_state(state, predicate);
        h[g] = topspin::getSolutionLength(abstraction, k);
    }
    return *std::max_element(h.begin(), h.end());
}

int modDistance(const std::vector<int>& state, int k, int mod) {
    std::vector<int> h(mod, 0);
    for (int m = 0; m < mod; ++m) {
        auto predicate = [m, mod](int x) { return x % mod == m; };
        std::vector<int> abstraction = topspin::abstract_state(state, predicate);
        h[m] = topspin::getSolutionLength(abstraction, k);
    }
    return *std::max_element(h.begin(), h.end());
}

int twoGroupC(const std::vector<int>& state, int k) {
    int bound = state.size() / 2;
        auto mapping = [bound](int x) {
        if (x <= bound) return 1;
        else return 2;
    };
    std::vector<int> abstraction = topspin::abstract_stateC(state, mapping);
    int h = topspin::getSolutionLengthC(abstraction, k, mapping);
    return h;
}

int threeGroupC(const std::vector<int>& state, int k) {
    int bound = state.size() / 3;
    auto mapping = [bound](int x) {
        if (x <= bound) return 1;
        else if (x > bound && x <= 2 * bound) return 2;
        else return 3;
    };
    std::vector<int> abstraction = topspin::abstract_stateC(state, mapping);
    int h = topspin::getSolutionLengthC(abstraction, k, mapping);
    return h;
}

int fourGroupC(const std::vector<int>& state, int k) {
    int bound = state.size() / 4;
    auto mapping = [bound](int x) {
        if (x <= bound) return 1;
        else if (x > bound && x <= 2 * bound) return 2;
        else if (x > 2 * bound && x <= 3 * bound) return 3;
        else return 4;
    };
    std::vector<int> abstraction = topspin::abstract_stateC(state, mapping);
    int h = topspin::getSolutionLengthC(abstraction, k, mapping);
    return h;
}

int oddEvenC(const std::vector<int>& state, int k) {
    auto mapping = [](int x) { return x % 2; };
    std::vector<int> abstraction = topspin::abstract_stateC(state, mapping);
    int h = topspin::getSolutionLengthC(abstraction, k, mapping);
    return h;
}

int threeDistanceC(const std::vector<int>& state, int k) {
    auto mapping = [](int x) { return x % 3; };
    std::vector<int> abstraction = topspin::abstract_stateC(state, mapping);
    int h = topspin::getSolutionLengthC(abstraction, k, mapping);
    return h;
}

int fourDistanceC(const std::vector<int>& state, int k) {
    auto mapping = [](int x) { return x % 4; };
    std::vector<int> abstraction = topspin::abstract_stateC(state, mapping);
    int h = topspin::getSolutionLengthC(abstraction, k, mapping);
    return h;
}

}  // namespace topspin
