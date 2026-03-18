# UGV Path Planning using A* Algorithm

This project simulates an Unmanned Ground Vehicle (UGV) navigating a grid-based battlefield to find the shortest path from a start point to a goal while avoiding obstacles.

---

## Overview

- The environment is a 2D grid (e.g., 70×70)
- Each cell represents:
  - 0 → free space  
  - 1 → obstacle  
- Obstacles are randomly generated with different densities:
  - Low → 10%
  - Medium → 20%
  - High → 35%

---

## Algorithm Used: A*

The UGV uses the A* algorithm to compute the shortest path efficiently.

It works using:

- g(n): distance from start to current node  
- h(n): estimated distance to goal (heuristic)

f(n) = g(n) + h(n)


The algorithm always selects the node with the lowest f(n) value.

---

## How It Works

1. Generate a grid with random obstacles  
2. Define:
   - Start → (0, 0)  
   - Goal → (n-1, n-1)  
3. Run the A* algorithm  
4. Explore valid neighboring cells (including diagonals)  
5. Avoid obstacles while searching  
6. Reconstruct the shortest path once the goal is reached  
7. Display results and metrics  

---

## Measures of Effectiveness (MoE)

The system evaluates performance using:

- Path Length → total distance traveled  
- Straight-line Distance → ideal shortest distance  
- Optimality Ratio → path_length / straight_line_distance  
- Explored Nodes → number of visited nodes  
- Execution Time → computation time  

---

## Output

- Console output showing:
  - Whether a path was found  
  - Path length  
  - Optimality ratio  
  - Nodes explored  
  - Execution time  

- Graph visualization of:
  - Obstacles  
  - Start position  
  - Goal position  
  - Final path  

---

obstacle density can be changed in the code

run_simulation(size=70, density='low')    # options: low, medium, high

