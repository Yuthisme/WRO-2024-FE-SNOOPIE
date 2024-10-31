import rclpy
from rclpy.node import Node
from gpiozero import OutputDevice, PWMOutputDevice, AngularServo
from fe_interfaces.msg import MotorControl # Replace with your actual package name

class Motor:
    def __init__(self):
        # Initialize the motor control pins and PWM for speed control
        self.IN1 = OutputDevice(17)
        self.IN2 = OutputDevice(22)
        self.IN3 = OutputDevice(23)
        self.IN4 = OutputDevice(24)
        self.pwm1 = PWMOutputDevice(12)
        self.pwm2 = PWMOutputDevice(13)
        self.servo = AngularServo(14, min_angle=0, max_angle=60, min_pulse_width=0.5/1000, max_pulse_width=2.5/1000, initial_angle=None)



  
    def forward(self, speed):
        # Set the motor speed for forward direction
        self.pwm1.value = speed
        self.pwm2.value = speed
        self.IN1.value = False
        self.IN2.value = True
        self.IN3.value = True
        self.IN4.value = False

    def reverse(self, speed):
        # Set the motor speed for reverse direction
        self.pwm1.value = speed
        self.pwm2.value = speed
        self.IN1.value = True
        self.IN2.value = False
        self.IN3.value = False
        self.IN4.value = True

    def stop(self):
        # Stop the motor by setting PWM to zero
        self.pwm1.value = 0
        self.pwm2.value = 0
        # self.cleanup()

    def cleanup(self):
        # Cleanup all GPIO resources
        self.IN1.close()
        self.IN2.close()
        self.IN3.close()
        self.IN4.close()
        self.pwm1.close()
        self.pwm2.close()

    # def cleanup(self):
    #     # Cleanup all GPIO resources
    #     self.IN1.
    #     self.IN2.open()
    #     self.IN3.open()
    #     self.IN4.open()
    #     self.pwm1.open()
    #     self.pwm2.open()
    

class MotorControlNode(Node):
    def __init__(self):
        super().__init__('motor_control_node')
        self.motor = Motor()
        
        # Subscribe to the MotorControl message
        self.subscription = self.create_subscription(
            MotorControl,  # Message type
            'motor_control',  # Topic name
            self.motor_control_callback,  # Callback function
            10  # Queue size
        )
        self.get_logger().info("Motor control node started and subscribed to 'motor_control' topic")

    def motor_control_callback(self, msg: MotorControl):
        # Control the motor based on the received message
        if msg.stop:
            self.get_logger().info("Stopping the motor")
            self.motor.stop()
            self.motor.servo.angle = msg.angle
            self.get_logger().info(f"angle: {msg.angle}")
        elif msg.forward:
            self.get_logger().info(f"Moving forward with speed: {msg.speed}")
            self.motor.forward(msg.speed)
            self.motor.servo.angle = msg.angle
        elif msg.reverse:
            self.get_logger().info(f"Reversing with speed: {msg.speed}")
            self.motor.reverse(msg.speed)
            self.motor.servo.angle = msg.angle

    def destroy_node(self):
        # Ensure the motor is stopped when shutting down
        self.motor.cleanup()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    motor_control_node = MotorControlNode()

    try:
        rclpy.spin(motor_control_node)
    except KeyboardInterrupt:
        pass
    finally:
        motor_control_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
