import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import MarkerArray
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class VisualizationBridge(Node):
    def __init__(self):
        super().__init__('visualization_bridge')
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscribe to relevant topics
        self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.create_subscription(MarkerArray, 'environment', self.environment_callback, 10)

    def odom_callback(self, msg):
        # Broadcast transform from odom to base_link
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = 0.0
        t.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)

    def scan_callback(self, msg):
        # The LaserScan message can be directly visualized in RViz
        pass

    def environment_callback(self, msg):
        # The MarkerArray message can be directly visualized in RViz
        pass

def main(args=None):
    rclpy.init(args=args)
    visualization_bridge = VisualizationBridge()
    rclpy.spin(visualization_bridge)
    visualization_bridge.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()