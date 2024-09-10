import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import random

class EnvironmentSimulation(Node):
    def __init__(self):
        super().__init__('environment_simulation')
        self.publisher = self.create_publisher(MarkerArray, 'environment', 10)
        self.timer = self.create_timer(1.0, self.publish_environment)  # 1 Hz update rate

        self.obstacles = self.generate_obstacles(10)  # Generate 10 random obstacles

    def generate_obstacles(self, num_obstacles):
        obstacles = []
        for i in range(num_obstacles):
            obstacle = Marker()
            obstacle.header.frame_id = "map"
            obstacle.type = Marker.CUBE
            obstacle.action = Marker.ADD
            obstacle.id = i
            obstacle.scale.x = random.uniform(0.5, 2.0)
            obstacle.scale.y = random.uniform(0.5, 2.0)
            obstacle.scale.z = random.uniform(1.0, 3.0)
            obstacle.color.r = 1.0
            obstacle.color.g = 0.0
            obstacle.color.b = 0.0
            obstacle.color.a = 1.0
            obstacle.pose.position.x = random.uniform(-50.0, 50.0)
            obstacle.pose.position.y = random.uniform(-50.0, 50.0)
            obstacle.pose.position.z = obstacle.scale.z / 2
            obstacles.append(obstacle)
        return obstacles

    def publish_environment(self):
        marker_array = MarkerArray()
        marker_array.markers = self.obstacles
        self.publisher.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    environment_simulation = EnvironmentSimulation()
    rclpy.spin(environment_simulation)
    environment_simulation.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()