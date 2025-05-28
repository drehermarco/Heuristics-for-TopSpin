#ifndef TOPSPIN_ABSTRACTION_HPP
#define TOPSPIN_ABSTRACTION_HPP

#include <vector>
#include <functional>

namespace topspin {

std::vector<int> abstract_state(const std::vector<int>& input, const std::function<bool(const int&)>& predicate);

bool is_goal(const std::vector<int>& abstraction, const int predicate);

std::vector<int> reverseWindow(const std::vector<int>& state, int pos, int k);

int getSolutionLength(const std::vector<int>& abstraction, int k, const int predicate);

} // namespace topspin

#endif // TOPSPIN_ABSTRACTION_HPP