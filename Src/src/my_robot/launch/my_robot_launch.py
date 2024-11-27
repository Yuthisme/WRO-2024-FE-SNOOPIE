#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

'''
Parameter Description:
---
- Set laser scan directon: 
  1. Set counterclockwise, example: {'laser_scan_dir': True}
  2. Set clockwise,        example: {'laser_scan_dir': False}
- Angle crop setting, Mask data within the set angle range:
  1. Enable angle crop fuction:
    1.1. enable angle crop,  example: {'enable_angle_crop_func': True}
    1.2. disable angle crop, example: {'enable_angle_crop_func': False}
  2. Angle cropping interval setting:
  - The distance and intensity data within the set angle range will be set to 0.
  - angle >= 'angle_crop_min' and angle <= 'angle_crop_max' which is [angle_crop_min, angle_crop_max], unit is degress.
    example:
      {'angle_crop_min': 135.0}
      {'angle_crop_max': 225.0}
      which is [135.0, 225.0], angle unit is degress.
'''

def generate_launch_description():
  # LDROBOT LiDAR publisher node
  ldlidar_node = Node(
      package='ldlidar_stl_ros2',
      executable='ldlidar_stl_ros2_node',
      name='STL27L',
      output='screen',
      parameters=[
        {'product_name': 'LDLiDAR_STL27L'},
        {'topic_name': 'scan'},
        {'frame_id': 'base_laser'},
        {'port_name': '/dev/ttyUSB0'},
        {'port_baudrate': 921600},
        {'laser_scan_dir': False},
        {'enable_angle_crop_func': False},
        {'angle_crop_min': 0.0},
        {'angle_crop_max': 0.0}
      ]
  )

  # base_link to base_laser tf node
  bno055= Node(
    package='bno055_imu',
    executable='bno055',
    name='bno055',
    arguments=[]
  )

  get_angle= Node(
    package='lidar_ros2',
    executable='get_angle',
    name='get_angle',
    arguments=[]
  )

  motor_control = Node(
    package='motor_control',
    executable='motor_control',
    name='motor_control',
    arguments=[]
  )

  object_detection = Node(
    package='object_detection',
    executable='object_detection_publisher',
    name='object_detection',
    arguments=[]
  )
  auto = Node(
    package='my_robot',
    executable='my_robot',
    name='my_robot',
    output='screen',
    arguments=[]
  )




  # Define LaunchDescription variable
  ld = LaunchDescription()

  ld.add_action(ldlidar_node)
  ld.add_action(bno055)
  ld.add_action(get_angle)
  ld.add_action(motor_control)
  ld.add_action(object_detection)
  # ld.add_action(auto)


  return ld