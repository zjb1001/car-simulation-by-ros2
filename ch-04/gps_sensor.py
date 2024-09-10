import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
import numpy as np

class GPSSensor(Node):
    def __init__(self):
        super().__init__('gps_sensor')
        self.subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)
        self.publisher = self.create_publisher(NavSatFix, 'gps/fix', 10)
        self.timer = self.create_timer(1.0, self.publish_gps_data)  # 1 Hz update rate

        self.car_x = 0.0
        self.car_y = 0.0

        # GPS properties
        self.origin_lat = 37.7749  # San Francisco latitude
        self.origin_lon = -122.4194  # San Francisco longitude
        self.noise_std = 2.0  # GPS noise standard deviation (in meters)

    def odom_callback(self, msg):
        self.car_x = msg.pose.pose.position.x
        self.car_y = msg.pose.pose.position.y

    def publish_gps_data(self):
        gps_fix = NavSatFix()
        gps_fix.header.stamp = self.get_clock().now().to_msg()
        gps_fix.header.frame_id = "gps_link"

        # Convert local coordinates to GPS coordinates (simplified)
        lat, lon = self.local_to_gps(self.car_x, self.car_y)

        # Add some noise to simulate GPS inaccuracy
        lat += np.random.normal(0, self.noise_std / 111000)  # Approx. 111km per degree of latitude
        lon += np.random.normal(0, self.noise_std / (111000 * np.cos(np.radians(lat))))

        gps_fix.latitude = lat
        gps_fix.longitude = lon
        gps_fix.altitude = 0.0  # Assuming flat ground for simplicity

        # Set covariance (3x3 matrix in row-major order)
        gps_fix.position_covariance = [
            self.noise_std**2, 0.0, 0.0,
            0.0, self.noise_std**2, 0.0,
            0.0, 0.0, 0.0
        ]
        gps_fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN

        self.publisher.publish(gps_fix)

    def local_to_gps(self, x, y):
        # Simplified conversion (assuming flat Earth and small distances)
        lat = self.origin_lat + (y / 111000)  # Approx. 111km per degree of latitude
        lon = self.origin_lon + (x / (111000 * np.cos(np.radians(self.origin_lat))))
        return lat, lon

def main(args=None):
    rclpy.init(args=args)
    gps_sensor = GPSSensor()
    rclpy.spin(gps_sensor)
    gps_sensor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()