import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import numpy as np
import math

class IMUSensor(Node):
    def __init__(self):
        super().__init__('imu_sensor')
        self.subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)
        self.publisher = self.create_publisher(Imu, 'imu/data', 10)
        self.timer = self.create_timer(0.01, self.publish_imu_data)  # 100 Hz update rate

        self.linear_velocity = [0.0, 0.0, 0.0]
        self.angular_velocity = [0.0, 0.0, 0.0]
        self.orientation = [0.0, 0.0, 0.0, 1.0]  # Quaternion [x, y, z, w]

        # IMU properties
        self.accelerometer_noise_std = 0.01  # m/s^2
        self.gyroscope_noise_std = 0.001  # rad/s
        self.gravity = 9.81  # m/s^2

    def odom_callback(self, msg):
        self.linear_velocity = [
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z
        ]
        self.angular_velocity = [
            msg.twist.twist.angular.x,
            msg.twist.twist.angular.y,
            msg.twist.twist.angular.z
        ]
        self.orientation = [
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ]

    def publish_imu_data(self):
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = "imu_link"

        # Set orientation (from odometry)
        imu_msg.orientation.x = self.orientation[0]
        imu_msg.orientation.y = self.orientation[1]
        imu_msg.orientation.z = self.orientation[2]
        imu_msg.orientation.w = self.orientation[3]

        # Set angular velocity (from odometry + noise)
        imu_msg.angular_velocity.x = self.angular_velocity[0] + np.random.normal(0, self.gyroscope_noise_std)
        imu_msg.angular_velocity.y = self.angular_velocity[1] + np.random.normal(0, self.gyroscope_noise_std)
        imu_msg.angular_velocity.z = self.angular_velocity[2] + np.random.normal(0, self.gyroscope_noise_std)

        # Calculate linear acceleration
        linear_acceleration = self.calculate_linear_acceleration()
        imu_msg.linear_acceleration.x = linear_acceleration[0] + np.random.normal(0, self.accelerometer_noise_std)
        imu_msg.linear_acceleration.y = linear_acceleration[1] + np.random.normal(0, self.accelerometer_noise_std)
        imu_msg.linear_acceleration.z = linear_acceleration[2] + np.random.normal(0, self.accelerometer_noise_std)

        # Set covariance matrices (simplified)
        imu_msg.orientation_covariance = [-1.0] * 9  # -1 indicates no orientation estimate
        imu_msg.angular_velocity_covariance = [self.gyroscope_noise_std**2] * 9
        imu_msg.linear_acceleration_covariance = [self.accelerometer_noise_std**2] * 9

        self.publisher.publish(imu_msg)

    def calculate_linear_acceleration(self):
        # This is a simplified calculation and doesn't account for all real-world factors
        # In a real IMU, acceleration would be measured in the sensor frame
        qx, qy, qz, qw = self.orientation
        
        # Rotate gravity vector by orientation quaternion
        gx = 2 * (qx*qz - qw*qy) * self.gravity
        gy = 2 * (qy*qz + qw*qx) * self.gravity
        gz = (qw*qw - qx*qx - qy*qy + qz*qz) * self.gravity

        # Add acceleration due to linear velocity change (assuming constant acceleration between updates)
        ax = (self.linear_velocity[0] - self.prev_linear_velocity[0]) / self.timer.timer_period_ns * 1e9
        ay = (self.linear_velocity[1] - self.prev_linear_velocity[1]) / self.timer.timer_period_ns * 1e9
        az = (self.linear_velocity[2] - self.prev_linear_velocity[2]) / self.timer.timer_period_ns * 1e9

        self.prev_linear_velocity = self.linear_velocity.copy()

        return [ax + gx, ay + gy, az + gz]

def main(args=None):
    rclpy.init(args=args)
    imu_sensor = IMUSensor()
    rclpy.spin(imu_sensor)
    imu_sensor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()