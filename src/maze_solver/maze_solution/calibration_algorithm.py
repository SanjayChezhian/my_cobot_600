import cv2
import numpy as np
import matplotlib.pyplot as plt
from queue import PriorityQueue

# Step 1: Load Camera Calibration

def load_camera_calibration(file_path):
    cv_file = cv2.FileStorage(file_path, cv2.FILE_STORAGE_READ)
    mtx = cv_file.getNode('K').mat()
    dist = cv_file.getNode('D').mat()
    cv_file.release()
    return mtx, dist

# Step 2: Detect the Maze Region
def detect_maze(image):
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
            maze_region = image[y:y + h, x:x + w]
            return maze_region, (x, y, w, h), image
    return None, None, None

# Step 3: Maze to Matrix
def maze_to_matrix(maze_image, output_grid_size=36):
    gray = cv2.cvtColor(maze_image, cv2.COLOR_BGR2GRAY)
    binary_maze = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 11, 6)
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
            grid_world[i, j] = 0 if np.any(cell == 0) else 1
    return grid_world

# Step 4: Heuristic for A* Algorithm
def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

# Step 5: A* Algorithm
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

# Step 6: Find Start and Goal
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
    return start, goal

# Step 7: Draw Path on Original Image
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

# Step 8: Main Function
def main():
    calibration_file = "calibration.yaml"  # Replace with your path
    K, D = load_camera_calibration(calibration_file)
    cap = cv2.VideoCapture(0)

     # Set camera resolution to avoid zoom effect
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    print("Press 's' to capture the maze image and start processing...")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to capture image from webcam.")
            break

        cv2.imshow("Webcam Feed", frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('s'):
            print("Image captured. Processing...")
            maze_image, bbox, original_image = detect_maze(frame)
            break
        elif key == ord('q'):
            print("Exiting...")
            cap.release()
            cv2.destroyAllWindows()
            return

    cap.release()
    cv2.destroyAllWindows()

    if maze_image is None:
        print("Maze not found!")
        return

    grid_world = maze_to_matrix(maze_image, output_grid_size=36)
    start, goal = get_start_goal(grid_world)
    if start is None or goal is None:
        print("Start or Goal position is blocked.")
        return

    path = astar(grid_world, start, goal)
    if path:
        print("Path found:", path)
        output_image = draw_path_on_image(original_image, path, bbox, grid_size=36, crop_padding=(0, 0))
        cv2.imshow("Path on Maze", output_image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    else:
        print("No path found.")

if __name__ == "__main__":
    main()
