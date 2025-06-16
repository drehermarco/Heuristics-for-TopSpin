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
        // Each reversal can affect up to (k - 1) positions of a tile,
        // so estimate number of moves required
        count += dist;
    }
    //return count;
    return static_cast<int>(std::ceil(count / static_cast<double>(k - 1)));
}

int gapHeuristic(const std::vector<int>& state, int k) {
    int n = static_cast<int>(state.size());
    int count = 0;
    for (int i = 0; i < n; i++) {
        int current = state[i];
        int next = state[(i + 1) % n];
        if (current == n && next == 1) {
            continue; // Skip the wrap-around case for circular
        }
        if (abs(current - next) > 1) {
            count++;
        }   
    }
    return static_cast<int>(std::ceil(count / 2.0));
}

int twoGroup(const std::vector<int>& state, int k) {
    int bound = state.size() / 2;
    std::vector<int> abstraction1 = topspin::abstract_state(state, [bound](int x) { return x <= bound; });
    std::vector<int> abstraction2 = topspin::abstract_state(state, [bound](int x) { return x > bound; });

    int h1 = topspin::getSolutionLength(abstraction1, k);
    int h2 = topspin::getSolutionLength(abstraction2, k);
    return std::max(h1, h2);
}

int threeGroup(const std::vector<int>& state, int k) {
    int bound = state.size() / 3;
    std::vector<int> abstraction1 = topspin::abstract_state(state, [bound](int x) { return x <= bound; });
    std::vector<int> abstraction2 = topspin::abstract_state(state, [bound](int x) { return x > bound && x <= 2 * bound; });
    std::vector<int> abstraction3 = topspin::abstract_state(state, [bound](int x) { return x > 2 * bound; });

    int h1 = topspin::getSolutionLength(abstraction1, k);
    int h2 = topspin::getSolutionLength(abstraction2, k);
    int h3 = topspin::getSolutionLength(abstraction3, k);
    return std::max({h1, h2, h3});
}

int fourGroup(const std::vector<int>& state, int k) {
    int bound = state.size() / 4;
    std::vector<int> abstraction1 = topspin::abstract_state(state, [bound](int x) { return x <= bound; });
    std::vector<int> abstraction2 = topspin::abstract_state(state, [bound](int x) { return x > bound && x <= 2 * bound; });
    std::vector<int> abstraction3 = topspin::abstract_state(state, [bound](int x) { return x > 2 * bound && x <= 3 * bound; });
    std::vector<int> abstraction4 = topspin::abstract_state(state, [bound](int x) { return x > 3 * bound; });

    int h1 = topspin::getSolutionLength(abstraction1, k);
    int h2 = topspin::getSolutionLength(abstraction2, k);
    int h3 = topspin::getSolutionLength(abstraction3, k);
    int h4 = topspin::getSolutionLength(abstraction4, k);
    return std::max({h1, h2, h3, h4});
}

int oddEven(const std::vector<int>& state, int k) {
    std::vector<int> abstraction1 = topspin::abstract_state(state, [](int x) { return x % 2 == 0; });
    std::vector<int> abstraction2 = topspin::abstract_state(state, [](int x) { return x % 2 == 1; });
    
    int h1 = topspin::getSolutionLength(abstraction1, k);
    int h2 = topspin::getSolutionLength(abstraction2, k);
    return std::max(h1, h2);
}

int threeDistance(const std::vector<int>& state, int k) {
    std::vector<int> abstraction1 = topspin::abstract_state(state, [](int x) { return x % 3 == 0; });
    std::vector<int> abstraction2 = topspin::abstract_state(state, [](int x) { return x % 3 == 1; });
    std::vector<int> abstraction3 = topspin::abstract_state(state, [](int x) { return x % 3 == 2; });

    int h1 = topspin::getSolutionLength(abstraction1, k);
    int h2 = topspin::getSolutionLength(abstraction2, k);
    int h3 = topspin::getSolutionLength(abstraction3, k);
    return std::max({h1, h2, h3});
}

int fourDistance(const std::vector<int>& state, int k) {
    std::vector<int> abstraction1 = topspin::abstract_state(state, [](int x) { return x % 4 == 0; });
    std::vector<int> abstraction2 = topspin::abstract_state(state, [](int x) { return x % 4 == 1; });
    std::vector<int> abstraction3 = topspin::abstract_state(state, [](int x) { return x % 4 == 2; });
    std::vector<int> abstraction4 = topspin::abstract_state(state, [](int x) { return x % 4 == 3; });

    int h1 = topspin::getSolutionLength(abstraction1, k);
    int h2 = topspin::getSolutionLength(abstraction2, k);
    int h3 = topspin::getSolutionLength(abstraction3, k);
    int h4 = topspin::getSolutionLength(abstraction4, k);
    return std::max({h1, h2, h3, h4});
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
