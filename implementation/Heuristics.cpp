#include "Heuristics.h"
#include "Abstraction.h"
#include <vector>
#include <functional>
#include <algorithm>
#include <iostream>
#include <cmath>

namespace topspin {

int gapHeuristic(const std::vector<int>& permutation, int k) {
    int n = static_cast<int>(permutation.size());
    int count = 0;
    for (int i = 0; i < n; i++) {
        int current = permutation[i];
        int next = permutation[(i + 1) % n];
        if (current == n && next == 1) {
            continue; // Skip the wrap-around case for circular permutation
        }
        if (abs(current - next) > 1) {
            count++;
        }   
    }
    return static_cast<int>(std::ceil(count / 2.0));
}

int abstractHeuristicTwoGroup(const std::vector<int>& permutation, int k) {
    int bound = permutation.size() / 2;
    std::vector<int> abstraction1 = topspin::abstract_state(permutation, [bound](int x) { return x <= bound; });
    std::vector<int> abstraction2 = topspin::abstract_state(permutation, [bound](int x) { return x > bound; });

    int h1 = topspin::getSolutionLength(abstraction1, k);
    int h2 = topspin::getSolutionLength(abstraction2, k);
    return std::max(h1, h2);
}

int abstractHeuristicThreeGroup(const std::vector<int>& permutation, int k) {
    int bound = permutation.size() / 3;
    std::vector<int> abstraction1 = topspin::abstract_state(permutation, [bound](int x) { return x <= bound; });
    std::vector<int> abstraction2 = topspin::abstract_state(permutation, [bound](int x) { return x > bound && x <= 2 * bound; });
    std::vector<int> abstraction3 = topspin::abstract_state(permutation, [bound](int x) { return x > 2 * bound; });

    int h1 = topspin::getSolutionLength(abstraction1, k);
    int h2 = topspin::getSolutionLength(abstraction2, k);
    int h3 = topspin::getSolutionLength(abstraction3, k);
    return std::max({h1, h2, h3});
}

int abstractHeuristicFourGroup(const std::vector<int>& permutation, int k) {
    int bound = permutation.size() / 4;
    std::vector<int> abstraction1 = topspin::abstract_state(permutation, [bound](int x) { return x <= bound; });
    std::vector<int> abstraction2 = topspin::abstract_state(permutation, [bound](int x) { return x > bound && x <= 2 * bound; });
    std::vector<int> abstraction3 = topspin::abstract_state(permutation, [bound](int x) { return x > 2 * bound && x <= 3 * bound; });
    std::vector<int> abstraction4 = topspin::abstract_state(permutation, [bound](int x) { return x > 3 * bound; });

    int h1 = topspin::getSolutionLength(abstraction1, k);
    int h2 = topspin::getSolutionLength(abstraction2, k);
    int h3 = topspin::getSolutionLength(abstraction3, k);
    int h4 = topspin::getSolutionLength(abstraction4, k);
    return std::max({h1, h2, h3, h4});
}

int abstractHeuristicOE(const std::vector<int>& permutation, int k) {
    std::vector<int> abstraction1 = topspin::abstract_state(permutation, [](int x) { return x % 2 == 0; });
    std::vector<int> abstraction2 = topspin::abstract_state(permutation, [](int x) { return x % 2 == 1; });
    
    int h1 = topspin::getSolutionLength(abstraction1, k);
    int h2 = topspin::getSolutionLength(abstraction2, k);
    return std::max(h1, h2);
}

int abstractHeuristicDistance3(const std::vector<int>& permutation, int k) {
    std::vector<int> abstraction1 = topspin::abstract_state(permutation, [](int x) { return x % 3 == 0; });
    std::vector<int> abstraction2 = topspin::abstract_state(permutation, [](int x) { return x % 3 == 1; });
    std::vector<int> abstraction3 = topspin::abstract_state(permutation, [](int x) { return x % 3 == 2; });

    int h1 = topspin::getSolutionLength(abstraction1, k);
    int h2 = topspin::getSolutionLength(abstraction2, k);
    int h3 = topspin::getSolutionLength(abstraction3, k);
    return std::max({h1, h2, h3});
}

int abstractHeuristicDistance4(const std::vector<int>& permutation, int k) {
    std::vector<int> abstraction1 = topspin::abstract_state(permutation, [](int x) { return x % 4 == 0; });
    std::vector<int> abstraction2 = topspin::abstract_state(permutation, [](int x) { return x % 4 == 1; });
    std::vector<int> abstraction3 = topspin::abstract_state(permutation, [](int x) { return x % 4 == 2; });
    std::vector<int> abstraction4 = topspin::abstract_state(permutation, [](int x) { return x % 4 == 3; });

    int h1 = topspin::getSolutionLength(abstraction1, k);
    int h2 = topspin::getSolutionLength(abstraction2, k);
    int h3 = topspin::getSolutionLength(abstraction3, k);
    int h4 = topspin::getSolutionLength(abstraction4, k);
    return std::max({h1, h2, h3, h4});
}

}  // namespace topspin
