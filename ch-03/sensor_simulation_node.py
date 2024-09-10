import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math
import random

class SensorSimulation(Node):
    def __init__(self):
        super().__init__('sensor_simulation')
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

    def odom_callback(self, msg):
        self.car_x = msg.pose.pose.position.x
        self.car_y = msg.pose.pose.position.y
        self.car_theta = 2 * math.atan2(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)

    def publish_lidar_data(self):
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'base_link'
        scan.angle_min = 0.0
        scan.angle_max = 2 * math.pi
        scan.angle_increment = (2 * math.pi) / 360
        scan.time_increment = 0.0
        scan.range_min = 0.1
        scan.range_max = 30.0

        # Simulate LiDAR data (simple random distances for demonstration)
        scan.ranges = [random.uniform(0.1, 30.0) for _ in range(360)]

        self.publisher.publish(scan)

def main(args=None):
    rclpy.init(args=args)
    sensor_simulation = SensorSimulation()
    rclpy.spin(sensor_simulation)
    sensor_simulation.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()