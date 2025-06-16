#ifndef TOPSPIN_HEURISTICS_H
#define TOPSPIN_HEURISTICS_H

#include <vector>
#include <functional>

namespace topspin {

// Manhatten like heuristic
int circularManhattanHeuristic(const std::vector<int>& permutation, int k);

// Gap heuristic
int gapHeuristic(const std::vector<int>& permutation, int k);

// Abstract heuristics (padding)
int twoGroup(const std::vector<int>& permutation, int k);
int threeGroup(const std::vector<int>& permutation, int k);
int fourGroup(const std::vector<int>& permutation, int k);
int oddEven(const std::vector<int>& state, int k);
int threeDistance(const std::vector<int>& state, int k);
int fourDistance(const std::vector<int>& state, int k);

// Abstract heuristics (Cartesian)
int twoGroupC(const std::vector<int>& permutation, int k);
int threeGroupC(const std::vector<int>& permutation, int k);
int fourGroupC(const std::vector<int>& permutation, int k);
int oddEvenC(const std::vector<int>& state, int k);
int threeDistanceC(const std::vector<int>& state, int k);
int fourDistanceC(const std::vector<int>& state, int k);


   
}  // namespace topspin

#endif // TOPSPIN_HEURISTICS_H
