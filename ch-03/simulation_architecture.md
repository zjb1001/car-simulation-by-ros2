# Chapter 3: Designing Simulation Architecture

## 3.1 Introduction

This chapter focuses on designing the simulation architecture for our car simulation project using ROS2. We will cover the node structure, communication flow, and integration with ROS2 tools and existing automotive systems.

## 3.2 Node Structure

We will design a modular architecture with the following main nodes:

1. CarModel
2. SensorSimulation
3. EnvironmentSimulation
4. VisualizationBridge

### 3.2.1 CarModel Node

The CarModel node will be responsible for simulating the car's physics and dynamics. It will:
- Receive control inputs (steering, throttle, brake)
- Update the car's state (position, velocity, orientation)
- Publish the car's state

### 3.2.2 SensorSimulation Node

This node will simulate various sensors commonly found in autonomous vehicles:
- LiDAR
- Cameras
- GPS
- IMU (Inertial Measurement Unit)

### 3.2.3 EnvironmentSimulation Node

This node will handle the simulation of the environment, including:
- Road network
- Static obstacles
- Dynamic obstacles (other vehicles, pedestrians)
- Weather conditions

### 3.2.4 VisualizationBridge Node

This node will bridge our simulation with visualization tools like RViz, allowing us to:
- Visualize the car's position and orientation
- Display sensor data (e.g., LiDAR point clouds, camera images)
- Show the simulated environment

## 3.3 Communication Flow

We will use ROS2 topics and services for inter-node communication. Here's an overview of the main communication flows:

1. Control inputs -> CarModel
2. CarModel -> SensorSimulation (car state for sensor data generation)
3. EnvironmentSimulation -> SensorSimulation (environment data for sensor simulation)
4. CarModel, SensorSimulation, EnvironmentSimulation -> VisualizationBridge

## 3.4 Integration with ROS2 Tools

We will integrate our simulation with the following ROS2 tools:

1. RViz for visualization
2. rqt for parameter tuning and debugging
3. rosbag for recording and playback of simulation data

## 3.5 Implementation

Let's implement a basic version of our architecture to demonstrate the concept.

### 3.5.1 CarModel Node Implementation

We'll start by implementing the CarModel node:

```python
##File: /Users/Pz/Code/OwnProject/car-simulation-by-ros2/ch-03/car_model_node.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
import math

class CarModel(Node):
    def __init__(self):
        super().__init__('car_model')
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10)
        self.publisher = self.create_publisher(Odometry, 'odom', 10)
        self.timer = self.create_timer(0.1, self.update_state)  # 10 Hz update rate

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0

    def cmd_vel_callback(self, msg):
        self.linear_velocity = msg.linear.x
        self.angular_velocity = msg.angular.z

    def update_state(self):
        dt = 0.1  # Time step (same as timer period)
        self.x += self.linear_velocity * math.cos(self.theta) * dt
        self.y += self.linear_velocity * math.sin(self.theta) * dt
        self.theta += self.angular_velocity * dt

        # Create and publish Odometry message
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose = Pose()
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)
        odom.twist.twist = Twist()
        odom.twist.twist.linear.x = self.linear_velocity
        odom.twist.twist.angular.z = self.angular_velocity

        self.publisher.publish(odom)

def main(args=None):
    rclpy.init(args=args)
    car_model = CarModel()
    rclpy.spin(car_model)
    car_model.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()