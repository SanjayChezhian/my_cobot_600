import cv2
import numpy as np
import glob

# Define chessboard parameters: grid size and square dimension in meters
grid_cols, grid_rows = 9, 6  # Number of inner corners in chessboard grid
square_dim = 0.025  # Size of a square in meters

# Prepare 3D points for the chessboard corners in the calibration pattern
chessboard_3D_points = np.zeros((grid_cols * grid_rows, 3), dtype=np.float32)
chessboard_3D_points[:, :2] = np.mgrid[0:grid_cols, 0:grid_rows].T.reshape(-1, 2) * square_dim

# Containers for storing 3D points (object points) and 2D points (image points)
object_points_list = []
image_points_list = []

# Path to images for calibration
image_path_pattern = "/Home/Desktop/cobot600/src/Aruco_Markers/images/*.png"

# Process each image in the specified folder
for image_file in glob.glob(image_path_pattern):
    image = cv2.imread(image_file)
    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
    # Find chessboard corners in the grayscale image
    found, corners = cv2.findChessboardCorners(gray_image, (grid_cols, grid_rows), None)
    
    if found:
        # Append corresponding 3D and 2D points
        object_points_list.append(chessboard_3D_points)
        image_points_list.append(corners)

# Perform camera calibration
calibration_success, camera_matrix, distortion_coeffs, rotation_vectors, translation_vectors = cv2.calibrateCamera(
    object_points_list, image_points_list, gray_image.shape[::-1], None, None
)

if calibration_success:
    # Save calibration parameters to a YAML file
    output_file = "/Home/Desktop/cobot600/src/Aruco_Markers/calibration_chessboard.yaml"
    file_storage = cv2.FileStorage(output_file, cv2.FILE_STORAGE_WRITE)
    file_storage.write("K", camera_matrix)
    file_storage.write("D", distortion_coeffs)
    file_storage.release()
