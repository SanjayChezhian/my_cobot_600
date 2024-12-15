import cv2
import numpy as np

# Load the image
image_path = "/home/kris/maze_opencv/datasets/10.jpeg"  # Replace with your image path
image = cv2.imread(image_path)
import cv2

def resize_and_show(image, window_name="Image", scale_percent=50):
    """
    Resizes an image while maintaining the aspect ratio and displays it.

    Parameters:
        image (ndarray): The input image to be resized and displayed.
        window_name (str): The name of the window where the image will be displayed.
        scale_percent (int): The percentage of the original size to scale the image.
    """
    # Get the original dimensions of the image
    height, width = image.shape[:2]

    # Calculate new dimensions
    new_width = int(width * scale_percent / 100)
    new_height = int(height * scale_percent / 100)

    # Resize the image
    resized_image = cv2.resize(image, (new_width, new_height))

    # Display the resized image
    cv2.imshow(window_name, resized_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

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

# Loop through contours to find the maze
for contour in contours:
    # Approximate the contour to a polygon
    peri = cv2.arcLength(contour, True)
    approx = cv2.approxPolyDP(contour, 0.02 * peri, True)

    # If the contour has 4 vertices, it may be the maze region
    if len(approx) == 4:
        # Draw the bounding box around the maze
        cv2.drawContours(image, [approx], -1, (0, 255, 0), 5)
        break

# Display the results
#cv2.imshow("Maze Detected", image)
#cv2.imshow("Binary Image", binary)
resize_and_show(binary, "binary resized", scale_percent=50)
resize_and_show(image, "Maze Detected resized", scale_percent=50)

cv2.waitKey(0)
cv2.destroyAllWindows()
