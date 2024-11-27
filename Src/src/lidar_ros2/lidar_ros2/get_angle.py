import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from fe_interfaces.msg import Angle
import math
import numpy as np

class LaserScanProcessor(Node):
    def __init__(self):
        super().__init__('laser_scan_processor')
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',  # Topic name might vary
            self.process_scan,
            10
        )

        self.publisher = self.create_publisher(Angle, 'angles', 10)
        
        # Define the target angles in degrees, then convert to radians for processing
        self.angles_array = np.array([360, 90, 180, 270], dtype=np.float32)  # Adjust angles as needed

        self.target_angles_deg_mult = np.array(
            list(range(270, 360, 5))  # Right side: 270° to 360°
            + list(range(1, 90, 5))   # Right side: 0° to 90°
            + list(range(90, 185, 5)) # Left side: 90° to 180°
            + list(range(180, 270, 5)) # Left side: 180° to 270°
        )
        # self.target_angles_rad_mult = [math.radians(a) for a in self.target_angles_deg_mult]  # Convert to radians

        self.angle_msg = Angle()  # Initialize the Angle message

    def process_scan(self, msg):
        for i, angle_deg in enumerate(self.angles_array):
            target_angle = math.radians(angle_deg)  # Convert degrees to radians for indexing
            
            # Check if the target angle is within the laser scan range
            if not (msg.angle_min <= target_angle <= msg.angle_max):
                self.get_logger().info(f"Target angle {angle_deg}° is out of range.")
                continue

            # Calculate the index corresponding to the target angle
            index = int((target_angle - msg.angle_min) / msg.angle_increment)

            # Ensure the index is within the range array
            if 0 <= index < len(msg.ranges):
                range_at_angle = msg.ranges[index]
                
                # Check if the range value is within the sensor's valid range
                if msg.range_min <= range_at_angle <= msg.range_max:
                    self.get_logger().info(f"Range at {angle_deg}°: {range_at_angle} meters")
                    self.angle_msg.angles[i] = range_at_angle  # Assign to the Angle message
                else:
                    self.get_logger().info(f"Range at {angle_deg}° is out of sensor's range.")
                    self.angle_msg.angles[i] = float('nan')  # Set to NaN if out of range
            else:
                self.get_logger().info(f"Index for angle {angle_deg}° is out of range.")

        for i, angle_deg in enumerate(self.target_angles_deg_mult):
            target_angle = math.radians(angle_deg)  # Convert degrees to radians for indexing
            
            # Check if the target angle is within the laser scan range
            if not (msg.angle_min <= target_angle <= msg.angle_max):
                self.get_logger().info(f"Target angle {angle_deg}° is out of range.")
                continue

            # Calculate the index corresponding to the target angle
            index = int((target_angle - msg.angle_min) / msg.angle_increment)

            # Ensure the index is within the range array
            if 0 <= index < len(msg.ranges):
                range_at_angle = msg.ranges[index]
                
                # Check if the range value is within the sensor's valid range
                if msg.range_min <= range_at_angle <= msg.range_max:
                    self.get_logger().info(f"Range at {angle_deg}°: {range_at_angle} meters")
                    self.angle_msg.angle_mult[i] = range_at_angle  # Assign to the Angle message
                else:
                    self.get_logger().info(f"Range at {angle_deg}° is out of sensor's range.")
                    self.angle_msg.angle_mult[i] = float('nan')  # Set to NaN if out of range
            else:
                self.get_logger().info(f"Index for angle {angle_deg}° is out of range.")

        # Publish the updated angle message
        self.publisher.publish(self.angle_msg)

def main(args=None):
    rclpy.init(args=args)
    laser_scan_processor = LaserScanProcessor()
    rclpy.spin(laser_scan_processor)
    laser_scan_processor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
