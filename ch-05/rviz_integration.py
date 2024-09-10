import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import ColorRGBA

class RVizIntegrationNode(Node):
    def __init__(self):
        super().__init__('rviz_integration_node')
        self.marker_publisher = self.create_publisher(Marker, 'visualization_marker', 10)
        self.timer = self.create_timer(1.0, self.publish_car_marker)

    def publish_car_marker(self):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "car"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        marker.pose = Pose()
        marker.pose.position = Point(x=0.0, y=0.0, z=0.5)
        marker.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        marker.scale.x = 2.0
        marker.scale.y = 1.0
        marker.scale.z = 0.5

        marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)

        self.marker_publisher.publish(marker)
        self.get_logger().info('Published car marker to RViz')

def main(args=None):
    rclpy.init(args=args)
    node = RVizIntegrationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()