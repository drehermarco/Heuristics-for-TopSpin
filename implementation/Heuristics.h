#ifndef TOPSPIN_HEURISTICS_H
#define TOPSPIN_HEURISTICS_H

#include <vector>
#include <functional>

namespace topspin {

int gapHeuristic(const std::vector<int>& permutation, int k);

int abstractHeuristicTwoGroup(const std::vector<int>& permutation, int k);

int abstractHeuristicThreeGroup(const std::vector<int>& permutation, int k);

int abstractHeuristicFourGroup(const std::vector<int>& permutation, int k);

int abstractHeuristicOE(const std::vector<int>& state, int k);

int abstractHeuristicDistance3(const std::vector<int>& state, int k);

int abstractHeuristicDistance4(const std::vector<int>& state, int k);
   
}  // namespace topspin

#endif // TOPSPIN_HEURISTICS_H
