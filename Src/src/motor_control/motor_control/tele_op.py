import rclpy
from rclpy.node import Node
from fe_interfaces.msg import MotorControl  # Replace with your actual package name
import sys
import termios
import tty

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_node')
        self.publisher = self.create_publisher(MotorControl, 'motor_control', 10)
        self.get_logger().info("Teleop node started, ready to take user input")

        # Initial values for speed and angle
        self.speed = 0.5  # Default speed
        self.angle = 50.0  # Initial angle

    def get_key(self):
        """Capture key press from the terminal without waiting for Enter."""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key

    def send_motor_command(self, speed, forward, reverse, stop, angle):
        """Send motor control command."""
        msg = MotorControl()
        msg.speed = speed
        self.speed = speed
        msg.forward = forward
        msg.reverse = reverse
        msg.stop = stop
        msg.angle = angle
        self.publisher.publish(msg)
        self.get_logger().info(f'Sent command - Speed: {speed}, Forward: {forward}, Reverse: {reverse}, Stop: {stop}, Angle: {angle}')

    def run(self):
        """Main loop to capture input and send motor commands."""
        self.get_logger().info("Use 'w' to move forward, 's' to move reverse, 'x' to stop, 'a' to decrease angle, 'd' to increase angle, 'q' to quit.")
        
        try:
            while True:
                key = self.get_key()

                if key == 'w':
                    # Move forward
                    self.speed = 0.5 
                    self.send_motor_command(self.speed, forward=True, reverse=False, stop=False, angle=self.angle)
                elif key == 's':
                    # Move reverse
                    self.speed = 0.5 
                    self.send_motor_command(self.speed, forward=False, reverse=True, stop=False, angle=self.angle)
                elif key == 'x':
                    # Stop the motor
                    self.send_motor_command(speed=0.0, forward=False, reverse=False, stop=True, angle=self.angle)
                elif key == 'a':
                    # Decrease angle
                    
                    self.angle = max(0.0, self.angle - 1)  # Ensure angle doesn't go below 0
                    self.get_logger().info(f"Decreasing angle: {self.angle}")
                    self.send_motor_command(self.speed, forward=False, reverse=True, stop=False, angle=self.angle)
                elif key == 'd':
                    # Increase angle
                    
                    self.angle = min(100, self.angle + 1)  # Ensure angle doesn't go above 100
                    self.get_logger().info(f"Increasing angle: {self.angle}")
                    self.send_motor_command(self.speed, forward=False, reverse=True, stop=False, angle=self.angle)
                elif key == 'q':
                    # Quit the program
                    self.get_logger().info("Quitting teleop.")
                    break
                else:
                    self.get_logger().info(f"Unknown key: {key}")
        
        except KeyboardInterrupt:
            pass

def main(args=None):
    rclpy.init(args=args)
    teleop_node = TeleopNode()

    try:
        teleop_node.run()
    except KeyboardInterrupt:
        pass
    finally:
        teleop_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
