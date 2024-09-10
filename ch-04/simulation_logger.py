import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Image, NavSatFix, Imu
from geometry_msgs.msg import Twist
import csv
import os
from datetime import datetime

class SimulationLogger(Node):
    def __init__(self):
        super().__init__('simulation_logger')
        
        # Create log directory
        self.log_dir = os.path.join(
当然，我会继续完成 4.7 Observation and Logging 部分的内容。以下是 `simulation_logger.py` 文件的完整实现：

```python
##File: /Users/Pz/Code/OwnProject/car-simulation-by-ros2/ch-04/simulation_logger.py
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Image, NavSatFix, Imu
from geometry_msgs.msg import Twist
import csv
import os
from datetime import datetime

class SimulationLogger(Node):
    def __init__(self):
        super().__init__('simulation_logger')
        
        # Create log directory
        self.log_dir = os.path.join(os.getcwd(), 'simulation_logs')
        os.makedirs(self.log_dir, exist_ok=True)
        
        # Create log files
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.odom_log = open(os.path.join(self.log_dir, f'odom_log_{timestamp}.csv'), 'w', newline='')
        self.odom_writer = csv.writer(self.odom_log)
        self.odom_writer.writerow(['timestamp', 'x', 'y', 'theta', 'v_x', 'v_y', 'omega'])

        self.cmd_vel_log = open(os.path.join(self.log_dir, f'cmd_vel_log_{timestamp}.csv'), 'w', newline='')
        self.cmd_vel_writer = csv.writer(self.cmd_vel_log)
        self.cmd_vel_writer.writerow(['timestamp', 'linear_x', 'angular_z'])

        self.lidar_log = open(os.path.join(self.log_dir, f'lidar_log_{timestamp}.csv'), 'w', newline='')
        self.lidar_writer = csv.writer(self.lidar_log)
        self.lidar_writer.writerow(['timestamp'] + [f'range_{i}' for i in range(360)])

        self.gps_log = open(os.path.join(self.log_dir, f'gps_log_{timestamp}.csv'), 'w', newline='')
        self.gps_writer = csv.writer(self.gps_log)
        self.gps_writer.writerow(['timestamp', 'latitude', 'longitude', 'altitude'])

        self.imu_log = open(os.path.join(self.log_dir, f'imu_log_{timestamp}.csv'), 'w', newline='')
        self.imu_writer = csv.writer(self.imu_log)
        self.imu_writer.writerow(['timestamp', 'orientation_x', 'orientation_y', 'orientation_z', 'orientation_w',
                                  'angular_velocity_x', 'angular_velocity_y', 'angular_velocity_z',
                                  'linear_acceleration_x', 'linear_acceleration_y', 'linear_acceleration_z'])

        # Create subscriptions
        self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(LaserScan, 'scan', self.lidar_callback, 10)
        self.create_subscription(NavSatFix, 'gps/fix', self.gps_callback, 10)
        self.create_subscription(Imu, 'imu/data', self.imu_callback, 10)

    def odom_callback(self, msg):
        timestamp = self.get_clock().now().to_msg().sec
        self.odom_writer.writerow([
            timestamp,
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            2 * math.atan2(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w),
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.angular.z
        ])

    def cmd_vel_callback(self, msg):
        timestamp = self.get_clock().now().to_msg().sec
        self.cmd_vel_writer.writerow([
            timestamp,
            msg.linear.x,
            msg.angular.z
        ])

    def lidar_callback(self, msg):
        timestamp = self.get_clock().now().to_msg().sec
        self.lidar_writer.writerow([timestamp] + list(msg.ranges))

    def gps_callback(self, msg):
        timestamp = self.get_clock().now().to_msg().sec
        self.gps_writer.writerow([
            timestamp,
            msg.latitude,
            msg.longitude,
            msg.altitude
        ])

    def imu_callback(self, msg):
        timestamp = self.get_clock().now().to_msg().sec
        self.imu_writer.writerow([
            timestamp,
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w,
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z,
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ])

    def __del__(self):
        self.odom_log.close()
        self.cmd_vel_log.close()
        self.lidar_log.close()
        self.gps_log.close()
        self.imu_log.close()

def main(args=None):
    rclpy.init(args=args)
    simulation_logger = SimulationLogger()
    rclpy.spin(simulation_logger)
    simulation_logger.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()