#include "Abstraction.h"

#include <queue>
#include <set>
#include <string>
#include <unordered_set>
#include <unordered_map>
#include <algorithm>
#include <iostream>
#include <ranges>
#include <cstdint>

namespace topspin {

using State = std::vector<uint8_t>;
struct VecHash {
    size_t operator()(const State& v) const noexcept {
        size_t h = 0;
        for (uint8_t x : v) h ^= std::hash<uint8_t>{}(x) + 0x9e3779b9 + (h << 6) + (h >> 2);
        return h;
    }
};
static std::unordered_map<State, int, VecHash> solutionLengthCache;

std::vector<uint8_t> abstract_state(const std::vector<uint8_t>& input, const std::function<bool(const uint8_t&)>& predicate) {
    std::vector<uint8_t> abstraction;
    abstraction.reserve(input.size());
    for (const auto& item : input) {
        abstraction.push_back(predicate(item) ? item : 0);
    }
    return abstraction;
}

std::vector<uint8_t> abstract_stateC(const std::vector<uint8_t>& input, const std::function<int(const uint8_t&)>& mapping) {
    std::vector<uint8_t> abstraction(input.size());
    std::ranges::transform(input, abstraction.begin(), mapping);
    return abstraction;
}

std::vector<uint8_t> normalize(std::vector<uint8_t> state) {
    auto filtered = state | std::views::filter([](uint8_t x) { return x != 0; });
    uint8_t min_non_zero = *std::ranges::min_element(filtered);
    
    auto it = std::ranges::find(state, min_non_zero);
    int current_index = std::distance(state.begin(), it);
    int target_index = min_non_zero - 1;

    int shift = (target_index - current_index + state.size()) % state.size();

    std::rotate(state.begin(), state.begin() + (state.size() - shift), state.end());
    return state;
}

bool is_goal(const std::vector<uint8_t>& abstraction) {
    const int n = static_cast<int>(abstraction.size());
    std::vector<uint8_t> normalized = normalize(abstraction);
    for (int i = 0; i < n; i++) {
        if (normalized[i] == 0) {
            continue;
        } else if (normalized[i] != i + 1) {
            return false;
        }
    }
    return true;
}

bool is_goalC(const std::vector<uint8_t>& abstraction, const std::function<int(uint8_t)>& mapping) {
    const int n = static_cast<int>(abstraction.size());
    for (int rot = 0; rot < n; rot++) {
        bool goal = true;
        for (int i = 0; i < n; i++) {
            if (abstraction[(i + rot) % n] != mapping(i + 1)) {
                goal = false;
                break;
            }
        }
        if (goal) return true;
    }
    return false;
}

std::vector<uint8_t> subvec_wraparound(const std::vector<uint8_t>& vec, int pos, int len) {
    std::vector<uint8_t> result;
    int n = static_cast<int>(vec.size());
    result.reserve(len);
    for (int i = 0; i < len; i++) {
        result.push_back(vec[(pos + i) % n]);
    }
    return result;
}

bool non_zero(const std::vector<uint8_t>& state, int pos, int n, int k) {
    for (int i = 0; i < k; i++) {
        if (state[(pos + i) % n] != 0) return true;
    }
    return false;
}

std::vector<uint8_t> reverseWindow(const std::vector<uint8_t>& state, int pos, int k) {
    const int n = static_cast<int>(state.size());
    std::vector<uint8_t> newState = state;

    for (int i = 0; i < k / 2; i++) {
        int left  = (pos + i) % n;
        int right = (pos + k - 1 - i) % n;
        std::swap(newState[left], newState[right]);
    }
    return newState;
}

template<typename GoalFunc>
int getSolutionLengthGeneric(const std::vector<uint8_t>& abstraction, int k, GoalFunc is_goal_func) {
    State key = normalize(abstraction);

    auto it = solutionLengthCache.find(key);
    if (it != solutionLengthCache.end()) return it->second;

    if (is_goal_func(key)) {
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

            if (is_goal_func(next)) {
                int goal_depth = depth + 1;
                solutionLengthCache[key] = goal_depth;
                return goal_depth;
            }
            q.push({std::move(next), depth + 1});
        }
    }
    solutionLengthCache[key] = -1;
    return -1;
}

int getSolutionLength(const std::vector<uint8_t>& abstraction, int k) {
    return getSolutionLengthGeneric(abstraction, k, is_goal);
}

int getSolutionLengthC(const std::vector<uint8_t>& abstraction, int k, const std::function<int(uint8_t)>& mapping) {
    return getSolutionLengthGeneric(abstraction, k, [&](const std::vector<uint8_t>& state) {
        return is_goalC(state, mapping);
    });
}

} // namespace topspin
