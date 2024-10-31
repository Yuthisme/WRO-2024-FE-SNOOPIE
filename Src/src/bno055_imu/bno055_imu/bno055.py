#!/usr/bin/python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
import board
import adafruit_bno055
import numpy as np
import sys

# Setup for BNO055 IMU sensor
i2c = board.I2C()
sensor = adafruit_bno055.BNO055_I2C(i2c)

class ImuPublisher(Node):
    def __init__(self):
        super().__init__('imu_publisher')
        # Create a publisher for IMU messages
        self.publisher_ = self.create_publisher(Imu, 'imu/data', 10)
        # Define a timer to call the callback function at a fixed rate
        timer_period = 0.01  # seconds (100 Hz)
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info("IMU publisher started")

    def timer_callback(self):
        # Fetch sensor data
        gyro = np.array(sensor.gyro)  # Gyroscope data
        accel = np.array(sensor.acceleration)  # Accelerometer data
        euler = sensor.euler  # Euler angles

        if euler is None:
            self.get_logger().warn("No Euler data available")
            return

        # Prepare IMU message
        imu_msg = Imu()

        # Fill in the orientation data (Note: we need to convert Euler angles to Quaternion)
        imu_msg.orientation = self.euler_to_quaternion(euler[0], euler[1], euler[2])

        # Fill in the angular velocity (gyroscope)
        imu_msg.angular_velocity.x = gyro[0]
        imu_msg.angular_velocity.y = gyro[1]
        imu_msg.angular_velocity.z = gyro[2]

        # Fill in the linear acceleration
        imu_msg.linear_acceleration.x = accel[0]
        imu_msg.linear_acceleration.y = accel[1]
        imu_msg.linear_acceleration.z = accel[2]

        # Publish the IMU message
        self.publisher_.publish(imu_msg)

        # self.get_logger().info(f'Published IMU data: Orientation = {euler}, Gyro = {gyro}, Accel = {accel}')

    def euler_to_quaternion(self, roll, pitch, yaw):
        """
        Convert Euler angles (in degrees) to a Quaternion
        """
        roll = np.radians(roll)
        pitch = np.radians(pitch)
        yaw = np.radians(yaw)

        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        self.get_logger().info(f'Published IMU data: Orientation = {roll}')

        return Quaternion(x=roll, y=qy, z=qz, w=qw)

def main(args=None):
    rclpy.init(args=args)
    imu_publisher = ImuPublisher()

    try:
        rclpy.spin(imu_publisher)
    except KeyboardInterrupt:
        pass

    imu_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
