import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math
import numpy as np

class LiDARSensor(Node):
    def __init__(self):
        super().__init__('lidar_sensor')
        self.subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)
        self.publisher = self.create_publisher(LaserScan, 'scan', 10)
        self.timer = self.create_timer(0.1, self.publish_lidar_data)  # 10 Hz update rate

        self.car_x = 0.0
        self.car_y = 0.0
        self.car_theta = 0.0

        # LiDAR properties
        self.angle_min = 0.0
        self.angle_max = 2 * math.pi
        self.angle_increment = (2 * math.pi) / 360
        self.range_min = 0.1
        self.range_max = 30.0
        self.noise_std = 0.01  # Standard deviation for Gaussian noise

    def odom_callback(self, msg):
        self.car_x = msg.pose.pose.position.x
        self.car_y = msg.pose.pose.position.y
        self.car_theta = 2 * math.atan2(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)

    def publish_lidar_data(self):
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'base_link'
        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = self.angle_increment
        scan.time_increment = 0.0
        scan.range_min = self.range_min
        scan.range_max = self.range_max

        # Simulate LiDAR data
        ranges = []
        for i in range(360):
            angle = i * self.angle_increment
            distance = self.simulate_distance(angle)
            ranges.append(distance)

        scan.ranges = ranges
        self.publisher.publish(scan)

    def simulate_distance(self, angle):
        # This is a placeholder for more complex environment simulation
        # For now, we'll just return a random distance with some noise
        base_distance = np.random.uniform(self.range_min, self.range_max)
        noise = np.random.normal(0, self.noise_std)
        return max(self.range_min, min(base_distance + noise, self.range_max))

def main(args=None):
    rclpy.init(args=args)
    lidar_sensor = LiDARSensor()
    rclpy.spin(lidar_sensor)
    lidar_sensor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()