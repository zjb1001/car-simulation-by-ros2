import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import math
import time

class CarSimulation(Node):
    def __init__(self):
        super().__init__('car_simulation')
        self.publisher_ = self.create_publisher(Odometry, 'odom', 10)
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.velocity_callback,
            10)
        self.perf_logger = self.create_publisher(String, 'performance_logs', 10)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.vtheta = 0.0
        self.last_time = self.get_clock().now()
        self.last_log_time = time.time()

    def velocity_callback(self, msg):
        callback_start = time.time()
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        self.vx = msg.linear.x
        self.vy = msg.linear.y
        self.vtheta = msg.angular.z

        self.x += self.vx * dt * math.cos(self.theta) - self.vy * dt * math.sin(self.theta)
        self.y += self.vx * dt * math.sin(self.theta) + self.vy * dt * math.cos(self.theta)
        self.theta += self.vtheta * dt

        self.publish_odometry()
        callback_end = time.time()
        
        self.log_performance(callback_start, callback_end)

    def publish_odometry(self):
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.angular.z = self.vtheta
        self.publisher_.publish(odom)

    def log_performance(self, start, end):
        latency = (end - start) * 1000  # Convert to milliseconds
        current_time = time.time()
        if current_time - self.last_log_time >= 1.0:  # Log every second
            freq = 1.0 / (current_time - self.last_log_time)
            log_msg = f"Frequency: {freq:.2f} Hz, Latency: {latency:.2f} ms"
            self.perf_logger.publish(String(data=log_msg))
            self.last_log_time = current_time

def main(args=None):
    rclpy.init(args=args)
    car_simulation = CarSimulation()
    rclpy.spin(car_simulation)
    car_simulation.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()