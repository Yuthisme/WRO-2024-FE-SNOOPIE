import numpy as np
import cv2

# Hex color #7e4033
hex_color = '#7a9047'

# Convert hex to BGR
bgr_color = np.array([int(hex_color[i:i+2], 16) for i in (5, 3, 1)], dtype=np.uint8)

# Convert BGR to HSV
bgr_color_reshaped = bgr_color.reshape(1, 1, 3)
hsv_color = cv2.cvtColor(bgr_color_reshaped, cv2.COLOR_BGR2HSV)[0][0]

# Define a range around the target color in HSV
hue = hsv_color[0]
lower_bound = np.array([max(hue - 10, 0), 50, 50])  # Adjust saturation and value as needed
upper_bound = np.array([min(hue + 10, 255), 255, 255])

print("Lower Bound:", lower_bound)
print("Upper Bound:", upper_bound)
