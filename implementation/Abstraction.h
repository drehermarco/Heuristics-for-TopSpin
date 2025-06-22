#ifndef TOPSPIN_ABSTRACTION_HPP
#define TOPSPIN_ABSTRACTION_HPP

#include <vector>
#include <functional>
#include <cstdint>

namespace topspin {

std::vector<uint8_t> abstract_state(const std::vector<uint8_t>& input, const std::function<bool(const uint8_t&)>& predicate);
std::vector<uint8_t> normalize(std::vector<uint8_t> state);
bool is_goal(const std::vector<uint8_t>& abstraction);
std::vector<uint8_t> subvec_wraparound(const std::vector<uint8_t>& vec, int pos, int len);
bool non_zero(const std::vector<uint8_t>& state, int pos, int n, int k);
std::vector<uint8_t> reverseWindow(const std::vector<uint8_t>& state, int pos, int k);
int getSolutionLength(const std::vector<uint8_t>& abstraction, int k);

std::vector<uint8_t> abstract_stateC(const std::vector<uint8_t>& input, const std::function<int(const uint8_t&)>& mapping);
bool is_goalC(const std::vector<uint8_t>& abstraction, const std::function<int(uint8_t)>& mapping);
int getSolutionLengthC(const std::vector<uint8_t>& abstraction, int k, const std::function<int(uint8_t)>& mapping);


} // namespace topspin

#endif // TOPSPIN_ABSTRACTION_HPP