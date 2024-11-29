import cv2

# Open the default camera (index 0). Adjust index if needed for different cameras.
camera = cv2.VideoCapture(0)

if not camera.isOpened():
    print("Error: Unable to access the camera.")
    exit()

# Initialize image counter
image_counter = 0

# Set camera properties
camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

while True:
    # Capture a frame from the camera
    success, frame = camera.read()
    if not success:
        print("Error: Unable to capture a frame.")
        break

    # Verify the frame dimensions
    if frame is not None:
        print(frame.shape)  # Output the shape of the frame (should be height, width, channels)

    # Wait for a key press
    key = cv2.waitKey(5)
    if key == 27:  # Press 'Esc' key to quit
        break
    elif key == ord('s'):  # Press 's' key to save the current frame
        save_path = f'/home/kris/mycobot_pro_600_ws/src/aruco_ros/scripts/images/img{image_counter}.png'
        cv2.imwrite(save_path, frame)
        print(f"Image saved: {save_path}")
        image_counter += 1

    # Display the captured frame
    cv2.imshow('Captured Frame', frame)

# Release the camera and close all OpenCV windows
camera.release()
cv2.destroyAllWindows()
