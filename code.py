import numpy as np
import heapq
import time
import math
import matplotlib.pyplot as plt

# -----------------------------
# Grid and obstacle generation
# -----------------------------
def generate_grid(size=70, density='medium'):
    grid = np.zeros((size, size))

    if density == 'low':
        prob = 0.1
    elif density == 'medium':
        prob = 0.2
    else:  # high
        prob = 0.35

    for i in range(size):
        for j in range(size):
            if np.random.rand() < prob:
                grid[i][j] = 1  # obstacle

    return grid

# -----------------------------
# Heuristic (Euclidean)
# -----------------------------
def heuristic(a, b):
    return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

# -----------------------------
# A* Algorithm
# -----------------------------
def astar(grid, start, goal):
    rows, cols = grid.shape
    open_list = []
    heapq.heappush(open_list, (0, start))

    came_from = {}
    g_score = {start: 0}
    explored_nodes = 0

    directions = [(-1,0),(1,0),(0,-1),(0,1), (-1,-1),(1,1),(-1,1),(1,-1)]

    while open_list:
        _, current = heapq.heappop(open_list)
        explored_nodes += 1

        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            return path[::-1], explored_nodes

        for d in directions:
            neighbor = (current[0] + d[0], current[1] + d[1])

            if (0 <= neighbor[0] < rows and 
                0 <= neighbor[1] < cols and 
                grid[neighbor] == 0):

                tentative_g = g_score[current] + heuristic(current, neighbor)

                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score = tentative_g + heuristic(neighbor, goal)
                    heapq.heappush(open_list, (f_score, neighbor))

    return None, explored_nodes

# -----------------------------
# Visualization
# -----------------------------
def plot_grid(grid, path, start, goal):
    plt.imshow(grid, cmap='gray_r')
    
    if path:
        x, y = zip(*path)
        plt.plot(y, x, color='blue', linewidth=2, label="Path")

    plt.scatter(start[1], start[0], color='green', label="Start")
    plt.scatter(goal[1], goal[0], color='red', label="Goal")
    
    plt.legend()
    plt.title("UGV Path Planning (A*)")
    plt.show()

# -----------------------------
# Main simulation
# -----------------------------
def run_simulation(size=70, density='medium'):
    grid = generate_grid(size, density)

    start = (0, 0)
    goal = (size-1, size-1)

    grid[start] = 0
    grid[goal] = 0

    start_time = time.time()
    path, explored = astar(grid, start, goal)
    end_time = time.time()

    # -----------------------------
    # Measures of Effectiveness
    # -----------------------------
    if path:
        path_length = sum(
            heuristic(path[i], path[i+1]) for i in range(len(path)-1)
        )
    else:
        path_length = float('inf')

    straight_line = heuristic(start, goal)

    print("\n--- Measures of Effectiveness ---")
    print(f"Obstacle Density: {density}")
    print(f"Path Found: {'Yes' if path else 'No'}")
    print(f"Path Length: {path_length:.2f}")
    print(f"Straight-line Distance: {straight_line:.2f}")
    print(f"Optimality Ratio: {path_length/straight_line:.2f}")
    print(f"Explored Nodes: {explored}")
    print(f"Computation Time: {end_time - start_time:.4f} sec")

    plot_grid(grid, path, start, goal)


# -----------------------------
# Run Example
# -----------------------------
if __name__ == "__main__":
    run_simulation(size=70, density='medium')  # low, medium, high
