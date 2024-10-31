import cv2
import numpy as np
from matplotlib import pyplot as plt

# Initialize the webcam
cap = cv2.VideoCapture(0)

while True:
    # Capture frame-by-frame
    ret, frame = cap.read()
    
    if not ret:
        break

    # Convert image to RGB for display purposes
    image_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    # Convert to HSV for better color detection
    hsv_image = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Define color ranges for red and green in HSV
    lower_red = np.array([0, 50, 50])
    upper_red = np.array([15, 255, 255])

    lower_green = np.array([40, 40, 40])
    upper_green = np.array([80, 255, 255])

    # Create masks for red and green colors
    red_mask = cv2.inRange(hsv_image, lower_red, upper_red)
    green_mask = cv2.inRange(hsv_image, lower_green, upper_green)

    # Find contours for both red and green masks
    contours_red, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours_green, _ = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Set a minimum area threshold to filter noise
    min_area_red = 5000  # Adjust for large red objects
    min_area_green = 1000  # Adjust for large green objects

    # Filter red and green contours based on area
    contours_red_filtered = [cnt for cnt in contours_red if cv2.contourArea(cnt) > min_area_red]
    contours_green_filtered = [cnt for cnt in contours_green if cv2.contourArea(cnt) > min_area_green]

    # Draw rectangles around large detected red objects
    for contour in contours_red_filtered:
        x, y, w, h = cv2.boundingRect(contour)
        cv2.rectangle(image_rgb, (x, y), (x + w, y + h), (255, 0, 0), 2)

    # Draw rectangles around large detected green objects
    for contour in contours_green_filtered:
        x, y, w, h = cv2.boundingRect(contour)
        cv2.rectangle(image_rgb, (x, y), (x + w, y + h), (0, 255, 0), 2)

    # Display the result
    cv2.imshow("Color Detection", cv2.cvtColor(image_rgb, cv2.COLOR_RGB2BGR))

    # Break the loop on 'q' key press
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the webcam and close windows
cap.release()
cv2.destroyAllWindows()