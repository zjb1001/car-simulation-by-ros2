import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math

class TestScenario(Node):
    def __init__(self):
        super().__init__('test_scenario')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.publish_command)  # 10 Hz
        self.start_time = self.get_clock().now()

    def publish_command(self):
        elapsed_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        cmd = Twist()

        if elapsed_time < 5.0:
            # Drive forward for 5 seconds
            cmd.linear.x = 1.0
        elif elapsed_time < 10.0:
            # Turn left for 5 seconds
            cmd.angular.z = 0.5
        elif elapsed_time < 15.0:
            # Drive forward for 5 seconds
            cmd.linear.x = 1.0
        else:
            # Stop
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        self.publisher.publish(cmd)

        if elapsed_time >= 15.0:
            self.get_logger().info('Test scenario completed')
            self.timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    test_scenario = TestScenario()
    rclpy.spin(test_scenario)
    test_scenario.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()