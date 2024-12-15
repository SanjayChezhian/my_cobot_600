import cv2
import numpy as np
import matplotlib.pyplot as plt
from queue import PriorityQueue
from scipy.spatial.transform import Rotation as R
import math

# Step 1: Detect the Maze Region
def detect_maze(image_path, mtx, dst):
    # Load the image
    image = cv2.imread(image_path)
    original = image.copy()

    # Undistort the image using calibration parameters
    image = cv2.undistort(image, mtx, dst, None, mtx)

    # Convert to grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Apply Gaussian blur to remove noise
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)

    # Apply adaptive thresholding
    binary = cv2.adaptiveThreshold(blurred, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV, 11, 2)

    # Find contours
    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Sort contours by area (largest contour likely corresponds to the maze)
    contours = sorted(contours, key=cv2.contourArea, reverse=True)

    for contour in contours:
        # Approximate the contour to a polygon
        peri = cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, 0.02 * peri, True)

        # If the contour has 4 vertices, it may be the maze region
        if len(approx) == 4:
            # Get the bounding box of the maze
            x, y, w, h = cv2.boundingRect(approx)
            maze_region = original[y:y + h, x:x + w]
            return maze_region, (x, y, w, h), original

    return None, None, None

# Step 2: Convert Maze to Matrix
def maze_to_matrix(maze_image, output_grid_size=25):
    # Convert to grayscale
    gray = cv2.cvtColor(maze_image, cv2.COLOR_BGR2GRAY)

    # Binarize the image
    binary_maze = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 3, 6)

    # Get dimensions and cell sizes
    h, w = binary_maze.shape
    cell_height = h // output_grid_size
    cell_width = w // output_grid_size

    # Initialize grid-world matrix
    grid_world = np.zeros((output_grid_size, output_grid_size), dtype=int)

    for i in range(output_grid_size):
        for j in range(output_grid_size):
            # Define cell boundaries
            start_y = i * cell_height
            end_y = (i + 1) * cell_height if i < output_grid_size - 1 else h
            start_x = j * cell_width
            end_x = (j + 1) * cell_width if j < output_grid_size - 1 else w

            # Check for black pixels (walls)
            cell = binary_maze[start_y:end_y, start_x:end_x]
            if np.any(cell == 0):  # Wall
                grid_world[i, j] = 0
            else:
                grid_world[i, j] = 1  # Open space

    return grid_world

# Step 3: A* Algorithm
def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def astar(grid_world, start, goal):
    open_list = PriorityQueue()
    open_list.put((0, start))
    came_from = {}
    g_score = {start: 0}
    f_score = {start: heuristic(start, goal)}

    directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]

    while not open_list.empty():
        _, current = open_list.get()
        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            path.reverse()
            return path

        for direction in directions:
            neighbor = (current[0] + direction[0], current[1] + direction[1])
            if 0 <= neighbor[0] < grid_world.shape[0] and 0 <= neighbor[1] < grid_world.shape[1]:
                if grid_world[neighbor[0], neighbor[1]] == 1:
                    tentative_g_score = g_score[current] + 1
                    if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                        came_from[neighbor] = current
                        g_score[neighbor] = tentative_g_score
                        f_score[neighbor] = g_score[neighbor] + heuristic(neighbor, goal)
                        open_list.put((f_score[neighbor], neighbor))
    return None

# Step 4: Find Start and Goal Positions
def get_start_goal(grid_world):
    start = goal = None

    # Check the first column for start (excluding the top and bottom cells)
    for i in range(1, grid_world.shape[0] - 1):  # Skip the top and bottom cells
        if grid_world[i, 0] == 1:
            start = (i, 0)
            break

    # Check the last column for goal (excluding the top and bottom cells)
    for i in range(1, grid_world.shape[0] - 1):  # Skip the top and bottom cells
        if grid_world[i, -1] == 1:
            goal = (i, grid_world.shape[1] - 1)
            break

    return start, goal

# Step 5: Draw Path on Original Image
def draw_path_on_image(original_image, path, bbox, grid_size):
    x, y, w, h = bbox
    cell_height = h // grid_size
    cell_width = w // grid_size

    for (i, j) in path:
        center_x = int(x + j * cell_width + cell_width / 2)
        center_y = int(y + i * cell_height + cell_height / 2)
        cv2.circle(original_image, (center_x, center_y), radius=5, color=(0, 0, 255), thickness=-1)

    return original_image

# Step 6: Main Function
def main():
    # Load camera calibration parameters
    calibration_file = '/home/kris/maze_opencv/calibration_chessboard.yaml'
    cv_file = cv2.FileStorage(calibration_file, cv2.FILE_STORAGE_READ)
    mtx = cv_file.getNode('K').mat()
    dst = cv_file.getNode('D').mat()
    cv_file.release()

    # Define image path
    image_path = "/home/kris/maze_opencv/datasets/10.jpeg"  # Replace with your image path

    # Detect maze and extract region
    maze_image, bbox, original_image = detect_maze(image_path, mtx, dst)
    if maze_image is None:
        print("Maze not found!")
        return

    # Convert maze to matrix
    grid_world = maze_to_matrix(maze_image, output_grid_size=25)

    # Define start and goal positions
    start, goal = get_start_goal(grid_world)
    if start is None or goal is None:
        print("Start or Goal position is blocked.")
        return

    # Solve maze using A* algorithm
    path = astar(grid_world, start, goal)
    if path:
        print("Path found:", path)
        for (x, y) in path:
            grid_world[x, y] = 2  # Mark path on grid

        # Visualize the grid with path
        plt.imshow(grid_world, cmap="nipy_spectral")
        plt.title("Grid with Path (2 = Path)")
        plt.axis('off')
        plt.show()

        # Draw the path on the original image
        output_image = draw_path_on_image(original_image, path, bbox, grid_size=25)

        # Display the result
        cv2.imshow("Path on Maze", output_image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    else:
        print("No path found.")

# Run the program
if __name__ == "__main__":
    main()
