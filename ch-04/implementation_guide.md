# Chapter 4: Implementing Core Simulation Functionality

## 4.1 Introduction

This chapter focuses on implementing the core functionality of our car simulation using ROS2. We will cover the creation of a basic car model, implementation of simulated sensors, and development of a basic environment simulation.

## 4.2 Detailed Implementation Plan

1. Create Basic Car Model
   - Implement kinematic model
   - Implement dynamic model
   - Create ROS2 node for car model

2. Implement Simulated Sensors
   - LiDAR sensor
   - Camera sensor
   - GPS sensor
   - IMU sensor

3. Develop Basic Environment Simulation
   - Create road network
   - Implement static obstacles
   - Implement dynamic obstacles (basic)

4. Integration and Testing
   - Integrate car model with sensors
   - Integrate environment with car and sensors
   - Create test scenarios
   - Implement observation and logging mechanisms

## 4.3 Basic Car Model Implementation

We'll start by implementing a more advanced car model that includes both kinematic and dynamic properties.

### 4.3.1 Kinematic and Dynamic Car Model

Let's create a new file for our advanced car model:

```python
##File: /Users/Pz/Code/OwnProject/car-simulation-by-ros2/ch-04/advanced_car_model.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
import math

class AdvancedCarModel(Node):
    def __init__(self):
        super().__init__('advanced_car_model')
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10)
        self.publisher = self.create_publisher(Odometry, 'odom', 10)
        self.timer = self.create_timer(0.05, self.update_state)  # 20 Hz update rate

        # Kinematic properties
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.v = 0.0  # linear velocity
        self.omega = 0.0  # angular velocity

        # Dynamic properties
        self.mass = 1500.0  # kg
        self.moment_of_inertia = 2500.0  # kg*m^2
        self.wheel_base = 2.7  # m
        self.max_steer_angle = 0.6  # radians
        self.max_acceleration = 3.0  # m/s^2
        self.max_deceleration = 8.0  # m/s^2

        # Control inputs
        self.throttle = 0.0
        self.steering = 0.0

    def cmd_vel_callback(self, msg):
        self.v = msg.linear.x
        self.omega = msg.angular.z

    def update_state(self):
        dt = 0.05  # Time step (same as timer period)

        # Update position and orientation (kinematic model)
        self.x += self.v * math.cos(self.theta) * dt
        self.y += self.v * math.sin(self.theta) * dt
        self.theta += self.omega * dt

        # Update velocity (dynamic model)
        acceleration = self.throttle * self.max_acceleration
        if self.throttle < 0:
            acceleration = self.throttle * self.max_deceleration
        self.v += acceleration * dt

        # Update angular velocity (dynamic model)
        steer_angle = self.steering * self.max_steer_angle
        self.omega = self.v * math.tan(steer_angle) / self.wheel_base

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
        odom.twist.twist.linear.x = self.v
        odom.twist.twist.angular.z = self.omega

        self.publisher.publish(odom)

def main(args=None):
    rclpy.init(args=args)
    advanced_car_model = AdvancedCarModel()
    rclpy.spin(advanced_car_model)
    advanced_car_model.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()