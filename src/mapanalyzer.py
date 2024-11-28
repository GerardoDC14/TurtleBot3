import cv2
import numpy as np
import matplotlib.pyplot as plt
import yaml

def detect_three_sided_rooms(image_path, yaml_path):
    # Load the image in grayscale
    img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)

    if img is None:
        print(f"Error: Unable to load image at {image_path}")
        return

    # Read the yaml file
    with open(yaml_path, 'r') as file:
        map_metadata = yaml.safe_load(file)

    negate = map_metadata.get('negate', 0)
    occupied_thresh = map_metadata.get('occupied_thresh', 0.65)
    free_thresh = map_metadata.get('free_thresh', 0.25)

    # Normalize the image to [0, 1]
    img_norm = img.astype(np.float32) / 255.0

    # Create a binary image where walls are white (255) and free space is black (0)
    binary = np.zeros_like(img_norm, dtype=np.uint8)
    binary[img_norm >= occupied_thresh] = 255  # Walls
    binary[img_norm <= free_thresh] = 0        # Free space

    # Invert the binary image if negate is 1
    if negate == 1:
        binary = cv2.bitwise_not(binary)

    # Apply morphological operations
    kernel = np.ones((3, 3), np.uint8)
    binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel, iterations=1)
    binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel, iterations=1)

    # Edge detection using Canny
    edges = cv2.Canny(binary, 5, 30, apertureSize=3)

    # Find contours in the binary image
    contours, _ = cv2.findContours(
        binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )

    # Initialize a list to hold detected rooms
    rooms = []

    # Create a color version of the original image to draw rooms
    img_rooms = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

    for i, cnt in enumerate(contours):
        # Compute area and perimeter
        area = cv2.contourArea(cnt)
        perimeter = cv2.arcLength(cnt, True)

        # Approximate the contour to a polygon
        epsilon = 0.02 * perimeter
        approx = cv2.approxPolyDP(cnt, epsilon, True)

        # Print contour properties
        print(f"Contour {i}: Area = {area}, Perimeter = {perimeter}, Vertices = {len(approx)}")

        # Proceed if area is within desired range
        if area > 50:
            rooms.append(approx)
            # Draw the contour for visualization
            cv2.drawContours(img_rooms, [approx], -1, (0, 255, 0), 2)

    if rooms:
        print(f"Detected {len(rooms)} room(s).")
    else:
        print("No rooms detected.")

    # Display the image with detected rooms
    plt.figure(figsize=(10, 10))
    plt.imshow(cv2.cvtColor(img_rooms, cv2.COLOR_BGR2RGB))
    plt.title('Detected Rooms via Contours')
    plt.axis('off')
    plt.show()

if __name__ == "__main__":
    detect_three_sided_rooms(
        '/home/gerardo/turtlebot3_group6v2/src/mapLabB.pgm',
        '/home/gerardo/turtlebot3_group6v2/src/mapLabB.yaml'
    )
