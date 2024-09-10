import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
import cv2
import numpy as np
from cv_bridge import CvBridge

class CameraSensor(Node):
    def __init__(self):
        super().__init__('camera_sensor')
        self.subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)
        self.publisher = self.create_publisher(Image, 'camera/image_raw', 10)
        self.timer = self.create_timer(0.033, self.publish_camera_data)  # 30 Hz update rate

        self.car_x = 0.0
        self.car_y = 0.0
        self.car_theta = 0.0

        self.bridge = CvBridge()

        # Camera properties
        self.image_width = 640
        self.image_height = 480

    def odom_callback(self, msg):
        self.car_x = msg.pose.pose.position.x
        self.car_y = msg.pose.pose.position.y
        self.car_theta = 2 * math.atan2(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)

    def publish_camera_data(self):
        # Generate a simple simulated image
        image = self.generate_simulated_image()

        # Convert the image to a ROS message
        img_msg = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
        img_msg.header.stamp = self.get_clock().now().to_msg()
        img_msg.header.frame_id = "camera_link"

        self.publisher.publish(img_msg)

    def generate_simulated_image(self):
        # Create a blank image
        image = np.zeros((self.image_height, self.image_width, 3), dtype=np.uint8)

        # Add a simple road
        cv2.rectangle(image, (0, self.image_height//2), (self.image_width, self.image_height), (100, 100, 100), -1)

        # Add lane markings
        for i in range(0, self.image_width, 50):
            cv2.line(image, (i, self.image_height//2 + self.image_height//4), 
                     (i+30, self.image_height//2 + self.image_height//4), (255, 255, 255), 5)

        # Add some "noise" to make the image more realistic
        noise = np.random.randint(0, 50, (self.image_height, self.image_width, 3), dtype=np.uint8)
        image = cv2.add(image, noise)

        return image

def main(args=None):
    rclpy.init(args=args)
    camera_sensor = CameraSensor()
    rclpy.spin(camera_sensor)
    camera_sensor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()