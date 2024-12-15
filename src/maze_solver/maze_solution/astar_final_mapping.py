import cv2
import numpy as np
import matplotlib.pyplot as plt
from queue import PriorityQueue

def detect_maze(image_path):
    image = cv2.imread(image_path)
    original = image.copy()
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    binary = cv2.adaptiveThreshold(blurred, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV, 11, 2)
    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = sorted(contours, key=cv2.contourArea, reverse=True)
    for contour in contours:
        peri = cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, 0.02 * peri, True)
        if len(approx) == 4:
            x, y, w, h = cv2.boundingRect(approx)
            maze_region = original[y:y+h, x:x+w]
            return maze_region, (x, y, w, h), original
    return None, None, None

def maze_to_matrix(maze_image, output_grid_size=22):
    gray = cv2.cvtColor(maze_image, cv2.COLOR_BGR2GRAY)
    binary_maze = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 3, 6)
    h, w = binary_maze.shape
    cell_height = h // output_grid_size
    cell_width = w // output_grid_size
    grid_world = np.zeros((output_grid_size, output_grid_size), dtype=int)
    for i in range(output_grid_size):
        for j in range(output_grid_size):
            start_y = i * cell_height
            end_y = (i + 1) * cell_height if i < output_grid_size - 1 else h
            start_x = j * cell_width
            end_x = (j + 1) * cell_width if j < output_grid_size - 1 else w
            cell = binary_maze[start_y:end_y, start_x:end_x]
            if np.any(cell == 0):
                grid_world[i, j] = 0
            else:
                grid_world[i, j] = 1
    return grid_world

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

def get_start_goal(grid_world):
    start = goal = None
    for i in range(1, grid_world.shape[0] - 1):
        if grid_world[i, 0] == 1:
            start = (i, 0)
            break
    for i in range(1, grid_world.shape[0] - 1):
        if grid_world[i, -1] == 1:
            goal = (i, grid_world.shape[1] - 1)
            break
    for j in range(grid_world.shape[1]):
        if grid_world[0, j] == 1:
            goal = (0, j)
        if grid_world[-1, j] == 1:
            start = (grid_world.shape[0] - 1, j)
    return start, goal

def map_to_real_world(path, bbox, grid_size, center_real_world, real_world_dimensions):
    x, y, w, h = bbox
    cell_height = h / grid_size
    cell_width = w / grid_size
    real_cell_width = real_world_dimensions[0] / grid_size
    real_cell_height = real_world_dimensions[1] / grid_size
    real_path1 = []
    real_path = []
    for (i, j) in path:
        real_x = (j - grid_size // 2) * real_cell_width + center_real_world[0]
        real_y = (i - grid_size // 2) * real_cell_height + center_real_world[1]
        real_path1.append((real_x, real_y))
        mapped_x = -0.25-0.165-real_y 
        mapped_y = -real_x
        real_path.append((mapped_x, mapped_y))
    return real_path

def draw_path_on_image(original_image, path, bbox, grid_size, crop_padding):
    x, y, w, h = bbox
    cell_height = h // grid_size
    cell_width = w // grid_size
    for (i, j) in path:
        i += crop_padding[0]
        j += crop_padding[1]
        center_x = int(x + j * cell_width + cell_width / 2)
        center_y = int(y + i * cell_height + cell_height / 2)
        cv2.circle(original_image, (center_x, center_y), radius=5, color=(0, 0, 255), thickness=-1)
    return original_image

def save_waypoints_to_file(waypoints, file_path):
    """
    Save waypoints to a file in the format "x y 0.1 0.0 -1.57 0.0".

    Args:
        waypoints (list of tuples): List of (x, y) waypoints.
        file_path (str): Path to the output text file.
    """
    with open(file_path, 'w') as file:
        for x, y in waypoints:
            line = f"{x:.2f} {y:.2f} 0.1 0.0 -1.57 0.0\n"
            file.write(line)

# Modify the main function to include saving the waypoints.
def main():
    image_path = "/home/sanjay/Desktop/cobot600/src/maze_solver/maze_solution/datasets/16.jpeg"
    center_real_world = (0,0)
    #mapped_positions = (-0.25-0.165-map_to_real_world.real_y , -map_to_real_world.real_x)

    real_world_dimensions = (0.152, 0.152)  # Example: 2.2m x 2.2m maze
    maze_image, bbox, original_image = detect_maze(image_path)
    if maze_image is None:
        print("Maze not found!")
        return
    grid_world = maze_to_matrix(maze_image, output_grid_size=22)
    cropped_grid_world = grid_world[1:-1, 1:-1]
    print("Cropped Grid World Matrix:")
    print(cropped_grid_world)
    start, goal = get_start_goal(cropped_grid_world)
    if start is None or goal is None:
        print("Start or Goal position is blocked.")
        return
    path = astar(cropped_grid_world, start, goal)
    if path:
        real_path = map_to_real_world(path, bbox, 22, center_real_world, real_world_dimensions)
        print("Path in real-world coordinates:")
        print(real_path)
        crop_padding = (1, 1)
        output_image = draw_path_on_image(original_image, path, bbox, grid_size=22, crop_padding=crop_padding)
        plt.imshow(cv2.cvtColor(output_image, cv2.COLOR_BGR2RGB))
        plt.title("Path on Maze")
        plt.axis('off')
        plt.show()

        # Save the waypoints to a file
        output_file = "/home/sanjay/Desktop/cobot600/src/maze_solver/maze_solution/waypoints.txt"
        save_waypoints_to_file(real_path, output_file)
        print(f"Waypoints saved to {output_file}")
    else:
        print("No path found.")

if __name__ == "__main__":
    main()
