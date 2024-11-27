import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import cv2 as cv
import numpy as np

class ObjectDetectionPublisher(Node):

    def __init__(self):
        super().__init__('object_detection_publisher')
        self.publisher_ = self.create_publisher(Point, '/object_detection', 10)
        self.timer = self.create_timer(0.01, self.timer_callback)

        # Initialize the webcam
        self.cap = cv.VideoCapture(0)

        # HSV ranges
        self.lower_red1 = np.array([0, 50, 50])     # Lower red range
        self.upper_red1 = np.array([10, 255, 255])  # Lower red range
        self.lower_red2 = np.array([170, 50, 50])   # Upper red range
        self.upper_red2 = np.array([180, 255, 255]) # Upper red range

        self.lower_green = np.array([40, 50, 50])   # Green range
        self.upper_green = np.array([70, 255, 255]) # Green range

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().info('No frame captured')
            return

        hsv_image = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

        # Red mask with wrap-around handling
        red_mask1 = cv.inRange(hsv_image, self.lower_red1, self.upper_red1)
        red_mask2 = cv.inRange(hsv_image, self.lower_red2, self.upper_red2)
        red_mask = cv.bitwise_or(red_mask1, red_mask2)

        # Green mask
        green_mask = cv.inRange(hsv_image, self.lower_green, self.upper_green)

        # Combine masks
        combined_mask = cv.bitwise_or(red_mask, green_mask)

        # Process all objects in the combined mask
        self.detect_and_publish_combined(frame, combined_mask, red_mask, green_mask)

        # Display the frame
        # cv.imshow("Pillar Detection", frame)
        """
        uncomment this to see it on your monitor
        """
        # if cv.waitKey(1) & 0xFF == ord('q'):
        #     self.cap.release()
        #     cv.destroyAllWindows()

    def detect_and_publish_combined(self, frame, combined_mask, red_mask, green_mask):
        contours, _ = cv.findContours(combined_mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        min_area = 1000

        detected_objects = []  # List to store detected objects
        frame_height, frame_width = frame.shape[:2]  # Get the dimensions of the frame
        frame_center_x = frame_width // 2  # Get the horizontal center of the frame

        # Define the blue rectangle at the bottom of the frame
        rectangle_width = 100
        rectangle_height = 50
        rectangle_start_x = frame_center_x - rectangle_width // 2
        rectangle_end_x = frame_center_x + rectangle_width // 2
        rectangle_center_x = (rectangle_start_x + rectangle_end_x) // 2  # Center of the blue rectangle

        # Draw a blue reference rectangle at the bottom of the frame with fixed width
        cv.rectangle(frame, 
                    (rectangle_start_x, frame_height - rectangle_height), 
                    (rectangle_end_x, frame_height), 
                    (255, 0, 0), -1)
            

        for contour in contours:
            if cv.contourArea(contour) < min_area:
                continue

            # Determine bounding box and centroid
            x, y, w, h = cv.boundingRect(contour)
            centroid_x = x + w / 2
            centroid_y = y + h / 2

            # Determine the color of the object
            mask_roi = combined_mask[y:y + h, x:x + w]
            red_overlap = cv.countNonZero(cv.bitwise_and(mask_roi, red_mask[y:y + h, x:x + w]))
            green_overlap = cv.countNonZero(cv.bitwise_and(mask_roi, green_mask[y:y + h, x:x + w]))

            if red_overlap > green_overlap:
                color = "red"
                display_color = (0, 0, 255)
            else:
                color = "green"
                display_color = (0, 255, 0)

            # Estimate distance using the width of the bounding box
            distance = 1 / (w + 1e-6)  # Smaller width -> farther away

            # Draw a rectangle and vertical line for the detected object
            # cv.rectangle(frame, (x, y), (x + w, y + h), display_color, 2)
            cv.line(frame, (int(centroid_x), 0), (int(centroid_x), frame.shape[0]), display_color, 2)

            # Append detected object
            detected_objects.append((distance, centroid_x, centroid_y, color))
        self.get_logger().info(f"Detected {len(detected_objects)} objects.")

        # Publish each detected object
        detected_objects.sort(key=lambda obj: obj[0])

        # Publish data only for the closest object
        if detected_objects:
            closest_object = detected_objects[0]  # The closest object will be at index 0
            distance, centroid_x, centroid_y, color = closest_object

            # Calculate the horizontal distance from the center of the blue rectangle
            distance_from_rectangle_center = centroid_x - rectangle_center_x



            # Publish data
            msg = Point()
            if color == 'red':
                msg.x = 1.0
            elif color == 'green':
                msg.x = 0.0
            # else:
            #     msg.x = -1.0
            msg.y = distance
            msg.z = distance_from_rectangle_center  # Positive if left, negative if right
            self.publisher_.publish(msg)

            self.get_logger().info(f"Detected {color} pillar at x: {centroid_x}, "
                                f"distance from rectangle center: {distance_from_rectangle_center:.4f}")
        else:
            msg = Point()
            msg.x = -1.0
            msg.y = 0.0
            msg.z = 0.0
            self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
