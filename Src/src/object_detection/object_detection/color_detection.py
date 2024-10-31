#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import cv2 as cv
import numpy as np

class ObjectDetectionPublisher(Node):

    def __init__(self):
        super().__init__('object_detection_publisher')
        self.publisher_ = self.create_publisher(Point, '/object_detection', 10)
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Initialize the webcam
        self.cap = cv.VideoCapture(0)

        # Define color ranges for red and green in HSV
        self.lower_red = np.array([169,  50,  50])
        self.upper_red = np.array([189, 255, 255])

        # Lower Bound: [169  50  50]
        # Upper Bound: [189 255 255]

        self.lower_green = np.array([25, 50, 50])
        self.upper_green = np.array([45, 255, 255])

        # Lower Bound: [25 50 50]
        # Upper Bound: [ 45 255 255]   

        # message 
        self.msg_data = []
             

    def timer_callback(self):

        self.object_detection()
        self.stop_object_detection()

        if self.msg_data == []:
            return
        
        msg = Point()
        msg.x = float(self.msg_data[0])
        msg.y = float(self.msg_data[1])
        if self.msg_data[2] == "red":       
            msg.z = float(0)
        elif self.msg_data[2] == "green":
            msg.z = float(1)

        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg)

    def object_detection(self):
        # Capture frame-by-frame
        ret, frame = self.cap.read()
        
        if not ret:
            self.get_logger().info('No frame')

        # Convert image to RGB for display purposes
        image_rgb = cv.cvtColor(frame, cv.COLOR_BGR2RGB)

        # Convert to HSV for better color detection
        hsv_image = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

        # Create masks for red and green colors
        red_mask = cv.inRange(hsv_image, self.lower_red, self.upper_red)
        green_mask = cv.inRange(hsv_image, self.lower_green, self.upper_green)

        # Find contours for both red and green masks
        contours_red, _ = cv.findContours(red_mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        contours_green, _ = cv.findContours(green_mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

        # Set a minimum area threshold to filter noise
        min_area_red = 1000  # Adjust for large red objects
        min_area_green = 1000  # Adjust for large green objects

        # Filter red and green contours based on area
        contours_red_filtered = [cnt for cnt in contours_red if cv.contourArea(cnt) > min_area_red]
        contours_green_filtered = [cnt for cnt in contours_green if cv.contourArea(cnt) > min_area_green]

        # Draw rectangles around large detected red objects
        for contour in contours_red_filtered:
            x, y, w, h = cv.boundingRect(contour)
            mid_x = x + w/2
            mid_y = y + h/2
            self.msg_data = [mid_x, mid_y, "red"]
            cv.rectangle(image_rgb, (x, y), (x + w, y + h), (255, 0, 0), 2)

        # Draw rectangles around large detected green objects
        for contour in contours_green_filtered:
            x, y, w, h = cv.boundingRect(contour)
            mid_x = x + w/2
            mid_y = y + h/2
            self.msg_data = [mid_x, mid_y, "green"]
            cv.rectangle(image_rgb, (x, y), (x + w, y + h), (0, 255, 0), 2)

        # Display the result
        cv.imshow("Color Detection", cv.cvtColor(image_rgb, cv.COLOR_RGB2BGR))

        

    def stop_object_detection(self):
        # Break the loop on 'q' key press
        if cv.waitKey(1) & 0xFF == ord('q'):
            # Release the webcam and close windows
            self.cap.release()
            cv.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)

    object_detection_publisher = ObjectDetectionPublisher()

    rclpy.spin(object_detection_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    object_detection_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()