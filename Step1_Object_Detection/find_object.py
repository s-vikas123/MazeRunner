"""
find_object.py

Author: Sai Vikas Satyanarayana, Keerthi Panguluri
Date: 1/23/25

This Python script uses OpenCV to implement a real-time object detection 
based on a user-specified color range. The users select a color by clicking 
on the live video feed, dynamically updating the HSV range to detect objects 
of the selected color.

"""
import cv2
import numpy as np

# Callback function to update HSV range based on mouse click
def pick_color(event, x, y, flags, param):
    global hsv, lower_bound, upper_bound, picked_color_image
    if event == cv2.EVENT_LBUTTONDOWN:
        kernel_size = 25
        half_k = kernel_size // 2

        # getting the neighboring pixels around the clicked pixel
        y_min = max(0, y - half_k)
        y_max = min(hsv.shape[0], y + half_k + 1)
        x_min = max(0, x - half_k)
        x_max = min(hsv.shape[1], x + half_k + 1)

        neighborhood = hsv[y_min:y_max, x_min:x_max]
        neighborhood = cv2.GaussianBlur(neighborhood, (5, 5), 0)  # Smoother blurring

        # Calculate HSV range with a buffer
        lower_bound[:] = np.maximum(np.percentile(neighborhood.reshape(-1, 3), 5, axis=0) - [10, 40, 40], [0, 0, 0])
        upper_bound[:] = np.minimum(np.percentile(neighborhood.reshape(-1, 3), 95, axis=0) + [10, 40, 40], [179, 255, 255])

        # Create a preview image for the picked color
        median_hsv = np.median(neighborhood.reshape(-1, 3), axis=0).astype(np.uint8)
        picked_color_image = np.zeros((200, 200, 3), dtype=np.uint8)
        picked_color_image[:] = cv2.cvtColor(np.uint8([[median_hsv]]), cv2.COLOR_HSV2BGR)[0][0]

        print(f"HSV Range Updated: Lower={lower_bound}, Upper={upper_bound}")
        print(f"Median HSV: {median_hsv}")

# Initialize global variables
lower_bound = np.array([0, 0, 0], dtype=np.uint8)
upper_bound = np.array([179, 255, 255], dtype=np.uint8)
picked_color_image = np.zeros((200, 200, 3), dtype=np.uint8)

# Start video capture
cap = cv2.VideoCapture(0)
cv2.namedWindow("Frame")
cv2.setMouseCallback("Frame", pick_color)

while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to capture frame. Exiting...")
        break

    # Adjust contrast of the frame
    frame = cv2.convertScaleAbs(frame, alpha=1.3, beta=0)

    # Convert frame to HSV color space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Create a mask to detect pixels within the picked color range
    mask = cv2.inRange(hsv, lower_bound, upper_bound)

    # Apply Gaussian blur to smooth edges
    mask = cv2.GaussianBlur(mask, (5, 5), 0)

    # Apply morphological operations to clean up the mask
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)  # Fill gaps
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)  # Remove noise
    mask = cv2.dilate(mask, kernel, iterations=1)  # Expand mask to ensure full coverage

    # Detect contours in the cleaned mask
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # Draw bounding boxes around the largest contour and calculate the center
    output_frame = frame.copy()
    if contours:
        largest_contour = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest_contour) > 800:
            x, y, w, h = cv2.boundingRect(largest_contour)
            center_x, center_y = x + w // 2, y + h // 2

            # Draw the bounding box and center point of the object
            cv2.rectangle(output_frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.circle(output_frame, (center_x, center_y), 5, (255, 0, 0), -1)
            cv2.putText(output_frame, f"({center_x}, {center_y})", (x, y - 20), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

            # Print the object location (center of the object) to the terminal
            print(f"Object Location: ({center_x}, {center_y})")

    # Display the frames
    cv2.imshow("Frame", frame)
    cv2.imshow("Processed Mask", mask)
    cv2.imshow("Picked Color", picked_color_image)
    cv2.imshow("Final Output", output_frame)

    # Exit condition: Press 'Esc' to exit
    if cv2.waitKey(1) & 0xFF == 27:
        break

# Release resources
cap.release()
cv2.destroyAllWindows()



