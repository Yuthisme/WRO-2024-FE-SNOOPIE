import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math

class LaserScanProcessor(Node):
    def __init__(self):
        super().__init__('laser_scan_processor')
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',  # Topic name might vary
            self.process_scan,
            10
        )
        
    def process_scan(self, msg):
        # Define the specific angle you want to get (in radians)
        target_angle = math.radians(270)  # Example: 30 degrees
        
        # Ensure the angle is within the range of the laser scanner
        if not (msg.angle_min <= target_angle <= msg.angle_max):
            self.get_logger().info("Target angle is out of range.")
            return

        # Calculate the index for the target angle
        index = int((target_angle - msg.angle_min) / msg.angle_increment)
        
        # Retrieve the range at this angle, checking for NaN or out-of-range values
        if 0 <= index < len(msg.ranges):
            range_at_angle = msg.ranges[index]
            
            if msg.range_min <= range_at_angle <= msg.range_max:
                self.get_logger().info(f"Range at {index} degrees: {range_at_angle} meters")
            else:
                self.get_logger().info(f"degrees: {range_at_angle} meters")
                # self.get_logger().info("Range at target angle is out of sensor's range.")
        else:
            self.get_logger().info("Calculated index is out of range.")
        
def main(args=None):
    rclpy.init(args=args)
    laser_scan_processor = LaserScanProcessor()
    rclpy.spin(laser_scan_processor)
    laser_scan_processor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
