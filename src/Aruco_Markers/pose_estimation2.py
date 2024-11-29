import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R
import math

# Dictionary that was used to generate the ArUco marker
aruco_dictionary_name = "DICT_6x6_250"

# The different ArUco dictionaries built into the OpenCV library
ARUCO_DICT = {
    "DICT_6x6_250": cv2.aruco.DICT_6X6_250
}

# Side length of the ArUco marker in meters
aruco_marker_side_length = 0.025

# Calibration parameters YAML file
camera_calibration_parameters_filename = 'calibration_chessboard.yaml'


def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into Euler angles (roll, pitch, yaw).
    Roll is rotation around x, pitch around y, yaw around z (all counterclockwise).
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z  # in radians


def draw_axes(frame, mtx, dst, rvec, tvec, length=0.1):
    """
    Draw axes on the frame given rotation and translation vectors.
    :param frame: Image on which to draw
    :param mtx: Camera matrix
    :param dst: Distortion coefficients
    :param rvec: Rotation vector
    :param tvec: Translation vector
    :param length: Length of the axis to draw
    """
    axis_points = np.float32([[length, 0, 0], [0, length, 0], [0, 0, length]]).reshape(-1, 3)
    image_points, _ = cv2.projectPoints(axis_points, rvec, tvec, mtx, dst)

    origin = tuple(map(int, image_points[0].ravel()))
    x_axis = tuple(map(int, image_points[1].ravel()))
    y_axis = tuple(map(int, image_points[2].ravel()))
    z_axis = tuple(map(int, image_points[2].ravel()))

    cv2.line(frame, origin, x_axis, (0, 0, 255), 5)  # Red for x-axis
    cv2.line(frame, origin, y_axis, (0, 255, 0), 5)  # Green for y-axis
    cv2.line(frame, origin, z_axis, (255, 0, 0), 5)  # Blue for z-axis



# Load camera calibration parameters
cv_file = cv2.FileStorage(camera_calibration_parameters_filename, cv2.FILE_STORAGE_READ)
mtx = cv_file.getNode('K').mat()
dst = cv_file.getNode('D').mat()
cv_file.release()

# Load the ArUco dictionary
print(f"[INFO] Detecting '{aruco_dictionary_name}' markers...")
this_aruco_dictionary = cv2.aruco.getPredefinedDictionary(ARUCO_DICT[aruco_dictionary_name])


# Start the video stream
cap = cv2.VideoCapture(8)

if not cap.isOpened():
    print("[ERROR] Could not open video stream!")
    exit()

# Capture a single frame
ret, frame = cap.read()

if not ret:
    print("[ERROR] Could not capture a frame!")
    cap.release()
    exit()


# Detect ArUco markers in the image
corners, marker_ids, rejected = cv2.aruco.detectMarkers(
    frame, this_aruco_dictionary )

if marker_ids is not None:
    cv2.aruco.drawDetectedMarkers(frame, corners, marker_ids)

    # Get the pose of the markers
    rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
        corners, aruco_marker_side_length, mtx, dst)
    

    for i, marker_id in enumerate(marker_ids):
        transform_translation_x = tvecs[i][0][0]
        transform_translation_y = tvecs[i][0][1]
        transform_translation_z = tvecs[i][0][2]

        rotation_matrix = np.eye(4)
        rotation_matrix[0:3, 0:3] = cv2.Rodrigues(np.array(rvecs[i][0]))[0]
        r = R.from_matrix(rotation_matrix[0:3, 0:3])
        quat = r.as_quat()

        roll_x, pitch_y, yaw_z = euler_from_quaternion(*quat)
        roll_x, pitch_y, yaw_z = map(math.degrees, (roll_x, pitch_y, yaw_z))

        
        print(f"[INFO] Marker ID: {marker_id[0]}")
        print(f"Translation: x={transform_translation_x}, y={transform_translation_y}, z={transform_translation_z}")
        print(f"Euler Angles: roll={roll_x}, pitch={pitch_y}, yaw={yaw_z}")
        print('\n')
    
        mapped_positions0 = []

        mapped_x = -0.15-0.1651-transform_translation_y
        mapped_y = -0.25-transform_translation_x

        mapped_positions0.append([mapped_x, mapped_y])
        print(mapped_positions0)


        mapped_positions1 = []

        mapped_x1 = -0.1651+transform_translation_y
        mapped_y1 = -0.25-transform_translation_x

        mapped_positions1.append([mapped_x1, mapped_y1])
        print(mapped_positions1)




        draw_axes(frame, mtx, dst, rvecs[i], tvecs[i], 0.05)

else:
    print("[INFO] No markers detected in the captured frame.")

cv2.imshow('Captured Frame', frame)
cv2.waitKey(0)

cap.release()
cv2.destroyAllWindows()
