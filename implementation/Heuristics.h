#ifndef TOPSPIN_HEURISTICS_H
#define TOPSPIN_HEURISTICS_H

#include <vector>
#include <functional>
#include <cstdint>

namespace topspin {
    
// Manhatten like heuristic
int circularManhattanHeuristic(const std::vector<uint8_t>& permutation, int k);

// Gap heuristic
int gapHeuristic(const std::vector<uint8_t>& permutation, int k);

// Abstract heuristics (padding)
int groupHeuristic(const std::vector<uint8_t>& permutation, int k, int numGroups);
int modDistance(const std::vector<uint8_t>& permutation, int k, int mod);

// Abstract heuristics (Cartesian)
int groupHeuristicC(const std::vector<uint8_t>& permutation, int k, int numGroups);
int modDistanceC(const std::vector<uint8_t>& permutation, int k, int mod);

// Breakpoint heuristic
int breakpointCalculation(const std::vector<uint8_t>& state, int k);
int breakpointHeuristic(const std::vector<uint8_t>& state, int k);
   
}  // namespace topspin

#endif // TOPSPIN_HEURISTICS_H
