#include "Abstraction.h"

#include <queue>
#include <set>
#include <string>
#include <unordered_set>
#include <unordered_map>
#include <algorithm>
#include <iostream>
#include <ranges>

namespace topspin {

std::vector<int> abstract_state(const std::vector<int>& input, const std::function<bool(const int&)>& predicate) {
    std::vector<int> abstraction;
    abstraction.reserve(input.size());
    for (const auto& item : input) {
        abstraction.push_back(predicate(item) ? item : 0);
    }
    return abstraction;
}

//Normalize the vector by shifting elements to relative positions of smallest non-zero element
std::vector<int> normalize(std::vector<int> state) {
    auto filtered = state | std::views::filter([](int x) { return x != 0; });
    int min_non_zero = *std::ranges::min_element(filtered);
    
    auto it = std::ranges::find(state, min_non_zero);
    int current_index = std::distance(state.begin(), it);
    int target_index = min_non_zero - 1;

    // Compute rotation distance
    int shift = (target_index - current_index + state.size()) % state.size();

    // Rotate to move min_non_zero to target_index
    std::rotate(state.begin(), state.begin() + (state.size() - shift), state.end());
    return state;
}

// Check if the abstraction is a goal state by normalizing it and checking if it matches the expected goal state
bool is_goal(const std::vector<int>& abstraction) {
    const int n = static_cast<int>(abstraction.size());
    std::vector<int> normalized = normalize(abstraction);
    for (int i = 0; i < n; i++) {
        if (normalized[i] == 0) {
            continue;
        } else if (normalized[i] != i + 1) {
            return false;
        }
    }
    return true;
}

std::vector<int> subvec_wraparound(const std::vector<int>& vec, int pos, int len) {
    std::vector<int> result;
    int n = static_cast<int>(vec.size());
    result.reserve(len);
    for (int i = 0; i < len; i++) {
        result.push_back(vec[(pos + i) % n]);
    }
    return result;
}

bool non_zero(const std::vector<int>& state, int pos, int n, int k) {
    for (int i = 0; i < k; i++) {
        if (state[(pos + i) % n] != 0) return true;
    }
    return false;
}

std::vector<int> reverseWindow(const std::vector<int>& state, int pos, int k) {
    const int n = static_cast<int>(state.size());
    std::vector<int> newState = state;

    for (int i = 0; i < k / 2; i++) {
        int left  = (pos + i) % n;
        int right = (pos + k - 1 - i) % n;
        std::swap(newState[left], newState[right]);
    }
    return newState;
}

int getSolutionLength(const std::vector<int>& abstraction, int k) {
    using State = std::vector<int>;
    struct VecHash {
        size_t operator()(const State& v) const noexcept {
            size_t h = 0;
            for (int x : v) h ^= std::hash<int>{}(x) + 0x9e3779b9 + (h << 6) + (h >> 2);
            return h;
        }
    };

    State key = normalize(abstraction);
    static std::unordered_map<State, int, VecHash> solutionLengthCache;

    auto it = solutionLengthCache.find(key);
    if (it != solutionLengthCache.end()) return it->second;
 

    if (is_goal(key)) {
        solutionLengthCache[key] = 0;
        return 0;
    }

    std::queue<std::pair<State, int>> q;
    std::unordered_set<State, VecHash> visited;

    q.push({key, 0});
    visited.insert(key);

    const int n = key.size();

    while (!q.empty()) {
        auto [current, depth] = q.front();
        q.pop();

        for (int pos = 0; pos < n; pos++) {
            if (!non_zero(current, pos, n, k)) continue;

            State next = reverseWindow(current, pos, k);
            if (!visited.emplace(next).second) continue;

            if (is_goal(next)) {
                solutionLengthCache[key] = depth + 1;
                return depth + 1;
            }
            q.push({std::move(next), depth + 1});
        }
    }
    solutionLengthCache[key] = -1;
    return -1;
}

} // namespace topspin