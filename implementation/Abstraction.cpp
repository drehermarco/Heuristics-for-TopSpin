#include "Abstraction.h"

#include <queue>
#include <set>
#include <string>
#include <unordered_set>
#include <algorithm>
#include <iostream>

namespace topspin {

std::vector<int> abstract_state(const std::vector<int>& input, const std::function<bool(const int&)>& predicate) {
    std::vector<int> abstraction;
    abstraction.reserve(input.size());
    for (const auto& item : input) {
        abstraction.push_back(predicate(item) ? item : 0);
    }
    return abstraction;
}

/*Modes:
    -> 1 == close groups f. ex. {1, 2, 3, 4, 5}
    -> 2 == even/odd groups f. ex. {0, 2, 0, 4, 0} and {1, 0, 3, 0, 5}
    -> 3 == distance of 3 between elements f. ex. {1, 0 , 0, 4, 0, 0}, {0, 2, 0, 0, 5, 0} and {0, 0, 3, 0, 0, 6}
*/
bool is_goal(const std::vector<int>& abstraction, int predicate) {
    const int n = static_cast<int>(abstraction.size());
    switch (predicate) {
    case 1:
        for (int i = 0; i < n; i++) {
            int a = abstraction[i];
            int b = abstraction[(i + 1) % n];
            if (a == 0 || a == *std::max_element(abstraction.begin(), abstraction.end())) continue;
            if ((a + 1) % n != b) return false;
        }
        return true;
    case 2:
        for (int i = 0; i < n; i++) {
            int a = abstraction[i];
            int b = abstraction[(i + 2) % n];
            if (a == 0) continue;
            if (!((a + 2 == b) || (a == n && b == 2) || (a == n - 1 && b == 1))) return false;
        }
        return true;
    case 3:
        for (int i = 0; i < n; i++) {
            int a = abstraction[i];
            int b = abstraction[(i + 3) % n];
            if (a == 0) continue;
            if (!((a + 3 == b) || (a == n && b == 3) || (a == n - 1 && b == 2) || (a == n - 2 && b == 3))) return false;
        }
        return true;
    case 4:
        for (int i = 0; i < n; i++) {
            int a = abstraction[i];
            int b = abstraction[(i + 4) % n];
            if (a == 0) continue;
            if (!((a + 4 == b) || (a == n && b == 4) || (a == n - 1 && b == 3) || (a == n - 2 && b == 2) || (a == n - 3 && b == 1))) return false;
        }
        return true;
    default:
        break;
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

int getSolutionLength(const std::vector<int>& abstraction, int k, const int predicate) {
    if (is_goal(abstraction, predicate)) return 0;

    using State = std::vector<int>;
    struct VecHash { size_t operator()(const State& v) const noexcept {
        size_t h = 0; for (int x : v) h ^= std::hash<int>{}(x) + 0x9e3779b9 + (h<<6) + (h>>2); return h;
    }};
    std::queue<std::pair<State, int>> q;
    std::unordered_set<State, VecHash> visited;

    q.push({abstraction, 0});
    visited.insert(abstraction);

    const int n = abstraction.size();

    while (!q.empty()) {
        auto [current, depth] = q.front();
        q.pop();

        std::vector<int> windowNonZero(n);
        int cnt = 0;

        for (int pos = 0; pos < n; ++pos) {
            if (pos == 0) {
                for (int i = 0; i < k; i++)
                    cnt += current[i] != 0;
            } else {
                if (current[(pos - 1) % n] != 0) --cnt;
                if (current[(pos + k - 1) % n] != 0) ++cnt;
            }
            windowNonZero[pos] = cnt;
        }

        for (int pos = 0; pos < n; pos++) {
            if (windowNonZero[pos] == 0) continue;

            State next = reverseWindow(current, pos, k);
            if (!visited.emplace(next).second) continue;
            if (is_goal(next, predicate)) return depth + 1;
            q.push({std::move(next), depth + 1});
        }
    }
    return -1;
}

} // namespace topspin