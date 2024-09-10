import unittest
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import GetEntityState
from visualization_msgs.msg import Marker

class TestVisualization(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node('test_visualization_node')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_gazebo_car_spawn(self):
        client = self.node.create_client(GetEntityState, '/get_entity_state')
        while not client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('Service not available, waiting again...')

        request = GetEntityState.Request()
        request.name = 'car'

        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)

        self.assertIsNotNone(future.result())
        self.assertEqual(future.result().success, True)

    def test_rviz_marker_publish(self):
        marker_received = False

        def marker_callback(msg):
            nonlocal marker_received
            if msg.ns == 'car' and msg.id == 0:
                marker_received = True

        subscription = self.node.create_subscription(
            Marker,
            'visualization_marker',
            marker_callback,
            10
        )

        rclpy.spin_once(self.node, timeout_sec=5.0)

        self.assertTrue(marker_received)
        self.node.destroy_subscription(subscription)

if __name__ == '__main__':
    unittest.main()