#ifndef TOPSPIN_ABSTRACTION_HPP
#define TOPSPIN_ABSTRACTION_HPP

#include <vector>
#include <functional>

namespace topspin {

std::vector<int> abstract_state(const std::vector<int>& input, const std::function<bool(const int&)>& predicate);
std::vector<int> normalize(std::vector<int> state);
bool is_goal(const std::vector<int>& abstraction);
std::vector<int> subvec_wraparound(const std::vector<int>& vec, int pos, int len);
bool non_zero(const std::vector<int>& state, int pos, int n, int k);
std::vector<int> reverseWindow(const std::vector<int>& state, int pos, int k);
int getSolutionLength(const std::vector<int>& abstraction, int k);

std::vector<int> abstract_stateC(const std::vector<int>& input, const std::function<int(const int&)>& mapping);
bool is_goalC(const std::vector<int>& abstraction, const std::function<int(int)>& mapping);
int getSolutionLengthC(const std::vector<int>& abstraction, int k, const std::function<int(int)>& mapping);


} // namespace topspin

#endif // TOPSPIN_ABSTRACTION_HPP