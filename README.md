WRO 2024 SNOOPY
====

## 1. Introduction

Hello and welcome to the github repository of our team Snoopie. Our team has been working hard for this year's WRO competition, there has been many issues that we faced during these time. One of it being managing time from school to build and develop our robot. You can find more information of our project below in the content area.

*Furthermore*, we'd like to elaborate on what's WRO, especially Future Engineer category. WRO stands for ***World Robotic Olympiad***, According to their website [About WRO](https://wro-association.org/wro-foundation/):
> WRO Foundation is a 501c3 USA non-profit organization founded to support World Robot Olympiad (WRO) Association and its efforts to encourage as many young people as possible to take an interest in the fields of science, technology, engineering & mathematics (STEM) through educational robotic competitions and skills development activities in the United States and worldwide.

*Future Engineer* is one of the 4 category that WRO have, and it's main goal is to develop a self driving car that has the abilities to avoid obstacle and park on its own. This category focus on the use of computer vision, and sensor fusion to develop an open source vehicle. This category differ to other categories as our hardware and software choices are all free of choice, meaning we can pick whatever we want. However our robot has to follow a certain dimension and weight as required by the [FE Rules](https://wro-association.org/wp-content/uploads/WRO-2024-Future-Engineers-Self-Driving-Cars-General-Rules.pdf).

*More Info*: For more info about WRO you can go to the [Official WRO Website](https://wro-association.org/).


## 2. Contents

* `t-photos` 3 photos of our team including 1 with the coach (funny and normal)
* `v-photos` 6 photos of our car from all 6 sides
* `video` Youtube link to our car demonstration
* `schemes` contain our board design
* `src` our codes
* `models` 3D designs, and lazer cutting as such.
* `other` Component listing

## 3. The Robot

![Car robot](https://github.com/user-attachments/assets/24b0fea0-ff45-4f24-a534-08699cf7918e)


At the core of my self-driving robot car lies a Raspberry Pi 5, a powerful single-board computer that serves as the brain of the operation. It processes sensor data, executes algorithms, and controls the car's actuators. To perceive its environment, the car relies on two primary sensors: a LiDAR sensor and a camera.

The LiDAR sensor emits laser beams to measure distances to objects, providing accurate 3D point cloud data essential for obstacle detection and mapping. The camera, equipped with OpenCV, a robust computer vision library, processes images from the real world. It can identify objects, lanes, traffic signs, and other visual cues that guide the car's navigation.

ROS2, the Robot Operating System 2, acts as the central nervous system of the self-driving car. It provides a flexible framework for organizing and coordinating various software components, including nodes, topics, and services. Nodes perform specific tasks, such as sensor data processing, control algorithms, or communication with other nodes. Topics are channels for publishing and subscribing to messages, enabling data exchange between nodes. Services provide a request-response mechanism for nodes to interact and request services from each other.

OpenCV, the Open Source Computer Vision Library, empowers the car to "see" the world around it. It enables a wide range of computer vision techniques, including image processing, feature detection, object detection and tracking, and optical flow.

To integrate these components, a ROS2-based system is meticulously crafted. LiDAR data is captured and processed to generate a point cloud representation of the environment, while camera images are captured and pre-processed using OpenCV to enhance clarity and contrast. LiDAR-based perception uses point cloud data to detect obstacles and create a 3D map of the surroundings, while camera-based perception employs OpenCV to identify lanes, traffic signs, and other relevant visual cues.

Based on the sensor data and perception results, the car's control system makes decisions about steering, acceleration, and braking. Path planning algorithms determine the optimal trajectory to reach the desired destination while avoiding obstacles. The control system then sends commands to the car's actuators to execute the desired maneuvers.

## 4. The Components

At the core of our robot we have a rasberry pi 5 with 8GB of ram. This rasberry pi 5 process all the data sent to the robot by the other component such as the lidar, camera, motor, and IMU. Here are the listing and purpose:

### 1. IMU (Inertial Measurement Unit)
- A device that measures the orientation and acceleration of a moving object. In our self-driving robot, the IMU provides crucial information about the robot's orientation, angular velocity, and linear acceleration. This data is essential for vehicle state estimation, sensor fusion, and control systems.
### 2. LiDAR (Light Detection and Ranging)
- It emits laser beams to measure distances to objects in its surroundings. It provides a 3D point cloud representation of the environment, enabling the robot to perceive obstacles, terrain, and other relevant features. Key applications of LiDAR in your self-driving robot include obstacle detection and avoidance, mapping and localization, and terrain analysis.
### 3. Rasberry Pi 5 8GB RAM
- The Raspberry Pi 5 serves as the brain of our self-driving robot. It processes sensor data, executes control algorithms, and communicates with other components. Key functionalities of the Raspberry Pi 5 in your robot include sensor data processing, algorithm execution, communication, and machine learning.
### 4. Motor Driver
- A motor driver is an electronic circuit that controls the speed and direction of electric motors. It acts as an interface between the Raspberry Pi 5 and the motors, allowing the robot to move forward, and backward. Key features of a motor driver include power amplification, direction control, and speed control.
### 5. Motor
- Motors are the actuators that convert electrical energy into mechanical energy, enabling the robot to move. In our self-driving robot, motors are typically used to drive the wheels. Key characteristics of motors for self-driving robots include torque, speed, and efficiency.
### 6. Servo Motor
- A servo motor is a rotary actuator with a built-in position sensor. It allows precise control of the angle of rotation, making it ideal for steering mechanisms in self-driving robots. Key features of a servo motor include precision, and holding torque.

## Members
- Leangheng Vongchhayyuth 
- Bunchhoeun Rattanakboth
- Sreng Kimroathpiseth 

