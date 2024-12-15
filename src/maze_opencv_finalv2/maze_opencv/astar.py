import cv2
import numpy as np
import matplotlib.pyplot as plt
from queue import PriorityQueue

# Load the maze image
image_path = '/home/kris/maze_opencv/datasets/1.png'  # Replace with your image path
maze = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)

# Binarize the image using adaptive thresholding
binary_maze = cv2.adaptiveThreshold(maze, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 3, 6)

# Define the output grid size (32x32)
output_grid_size = 11
h, w = binary_maze.shape

# Calculate exact cell size
cell_height = h // output_grid_size
cell_width = w // output_grid_size

# Initialize grid world matrix
grid_world = np.zeros((output_grid_size, output_grid_size), dtype=int)

for i in range(output_grid_size):
    for j in range(output_grid_size):
        # Define cell boundaries
        start_y = i * cell_height
        end_y = (i + 1) * cell_height if i < output_grid_size - 1 else h
        start_x = j * cell_width
        end_x = (j + 1) * cell_width if j < output_grid_size - 1 else w
        
        # Extract the sub-cell and check for black pixels
        cell = binary_maze[start_y:end_y, start_x:end_x]
        
        if np.any(cell == 0):  # Black pixel exists (wall)
            grid_world[i, j] = 0  # Wall
        else:
            grid_world[i, j] = 1  # Open space

# Set NumPy print options to show the entire matrix
np.set_printoptions(threshold=np.inf)  # Display the whole matrix

# Print the grid world matrix
print("Modified 32x32 Grid World:")
print(grid_world)

# Plot the grid world
plt.imshow(grid_world, cmap='gray')
plt.title("32x32 Grid World (0 = Open Space, 1 = Wall)")
plt.axis('off')  # Hide axis
plt.show()

# A* Algorithm Implementation
def heuristic(a, b):
    # Manhattan distance heuristic
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def astar(grid_world, start, goal):
    # Open list (priority queue) to store nodes to explore
    open_list = PriorityQueue()
    open_list.put((0, start))  # (f_score, position)
    
    # Maps to track the parent of each node and scores
    came_from = {}
    g_score = {start: 0}
    f_score = {start: heuristic(start, goal)}

    # Directions for moving: (x, y) -> Right, Down, Left, Up
    directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]

    while not open_list.empty():
        _, current = open_list.get()

        # If goal is reached, reconstruct path
        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            path.reverse()  # Reverse path to get it from start to goal
            return path

        # Explore neighbors
        for direction in directions:
            neighbor = (current[0] + direction[0], current[1] + direction[1])
            if 0 <= neighbor[0] < output_grid_size and 0 <= neighbor[1] < output_grid_size:
                if grid_world[neighbor[0], neighbor[1]] == 1:  # Check if open space
                    tentative_g_score = g_score[current] + 1
                    if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                        came_from[neighbor] = current
                        g_score[neighbor] = tentative_g_score
                        f_score[neighbor] = g_score[neighbor] + heuristic(neighbor, goal)
                        open_list.put((f_score[neighbor], neighbor))

    return None  # No path found

# Define the start and goal positions based on your specified openings
start = (0, 4)  # Start position at (0,4)
goal = (10, 6)  # Goal position at (10,6)

# Check start and goal positions (only triggered if they're blocked)
if grid_world[start[0], start[1]] == 0 or grid_world[goal[0], goal[1]] == 0:
    print("Start or Goal position is blocked.")
else:
    # Run A* to find the path
    path = astar(grid_world, start, goal)

    if path:
        print("Path found:", path)
        # Visualize the path
        for (x, y) in path:
            grid_world[x, y] = 2  # Mark the path with 2

        # Plot the path in the grid world
        plt.imshow(grid_world, cmap='nipy_spectral')
        plt.title("Path in Grid World (2 = Path)")
        plt.axis('off')
        plt.show()
    else:
        print("No path found.")



