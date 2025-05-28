# Heuristics for TopSpin

This project implements heuristics for the TopSpin puzzle.

## Compilation & Execution

1. **Navigate to the implementation directory**:
   ```bash
   cd implementation
   ```

2. **Compile the program**:
   ```bash
   g++ AStarSearch.cpp Heuristics.cpp Abstraction.cpp TopSpinStateSpace.cpp -o test
   ```

3. **Run the executable**:
   ```bash
   ./test
   ```

## Files

- `AStarSearch.cpp` – Implements the A* search algorithm.
- `Heuristics.cpp` – Contains heuristic functions for evaluating states.
- `Abstraction.cpp` – Handles domain abstractions.
- `TopSpinStateSpace.cpp` – Defines the TopSpin puzzle's state space and operations.
