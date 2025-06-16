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
   # Usage: ./test N k m h
   # N = number of tokens
   # k = reversal size
   # m = number of random actions applied before search
   # h = heuristic to use (e.g., 'gap', 'manhattan', etc.)
   ./test 20 4 20 gap
   ```

## Heuristics
| Heuristic Name   | Description                           |
| ---------------- | ------------------------------------- |
| `gap`            | Counts gaps between adjacent tokens   |
| `manhattan`      | Circular Manhattan distance           |
| `twoGroup`       | Two-group abstraction heuristic       |
| `threeGroup`     | Three-group abstraction heuristic     |
| `fourGroup`      | Four-group abstraction heuristic      |
| `oddEven`        | Odd-even token separation heuristic   |
| `threeDistance`  | Distance-based 3-token heuristic      |
| `fourDistance`   | Distance-based 4-token heuristic      |
| `twoGroupC`      | Compressed version of twoGroup        |
| `threeGroupC`    | Compressed version of threeGroup      |
| `fourGroupC`     | Compressed version of fourGroup       |
| `oddEvenC`       | Compressed version of oddEven         |
| `threeDistanceC` | Compressed 3-token distance heuristic |
| `fourDistanceC`  | Compressed 4-token distance heuristic |

## Files

- `AStarSearch.cpp` – Implements the A* search algorithm.
- `Heuristics.cpp` – Contains heuristic functions for evaluating states.
- `Abstraction.cpp` – Handles domain abstractions.
- `TopSpinStateSpace.cpp` – Defines the TopSpin puzzle's state space and operations.
