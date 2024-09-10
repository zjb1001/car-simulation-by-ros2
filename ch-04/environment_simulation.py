import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
import random
import math

class EnvironmentSimulation(Node):
    def __init__(self):
        super().__init__('environment_simulation')
        self.publisher = self.create_publisher(MarkerArray, 'environment', 10)
        self.car_subscription = self.create_subscription(
            Odometry,
            'odom',
            self.car_position_callback,
            10)
        self.timer = self.create_timer(1.0, self.publish_environment)  # 1 Hz update rate

        self.car_position = Point(x=0.0, y=0.0, z=0.0)
        self.road_network = self.generate_road_network()
        self.static_obstacles = self.generate_static_obstacles(10)
        self.dynamic_obstacles = self.generate_dynamic_obstacles(5)

    def car_position_callback(self, msg):
        self.car_position = msg.pose.pose.position

    def generate_road_network(self):
        road_markers = []
        # Create a simple grid road network
        for i in range(-50, 51, 10):
            road_markers.append(self.create_road_marker(Point(x=-50.0, y=float(i), z=0.0),
                                                        Point(x=50.0, y=float(i), z=0.0)))
            road_markers.append(self.create_road_marker(Point(x=float(i), y=-50.0, z=0.0),
                                                        Point(x=float(i), y=50.0, z=0.0)))
        return road_markers

    def generate_static_obstacles(self, num_obstacles):
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

    def generate_dynamic_obstacles(self, num_obstacles):
        obstacles = []
        for i in range(num_obstacles):
            obstacle = Marker()
            obstacle.header.frame_id = "map"
            obstacle.type = Marker.CYLINDER
            obstacle.action = Marker.ADD
            obstacle.id = i + 100  # To differentiate from static obstacles
            obstacle.scale.x = 1.0
            obstacle.scale.y = 1.0
            obstacle.scale.z = 1.5
            obstacle.color.r = 0.0
            obstacle.color.g = 1.0
            obstacle.color.b = 0.0
            obstacle.color.a = 1.0
            obstacle.pose.position.x = random.uniform(-50.0, 50.0)
            obstacle.pose.position.y = random.uniform(-50.0, 50.0)
            obstacle.pose.position.z = obstacle.scale.z / 2
            obstacles.append(obstacle)
        return obstacles

    def create_road_marker(self, start, end):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.2  # Line width
        marker.color.r = 0.5
        marker.color.g = 0.5
        marker.color.b = 0.5
        marker.color.a = 1.0
        marker.points = [start, end]
        return marker

    def update_dynamic_obstacles(self):
        for obstacle in self.dynamic_obstacles:
            # Simple random movement
            obstacle.pose.position.x += random.uniform(-0.5, 0.5)
            obstacle.pose.position.y += random.uniform(-0.5, 0.5)
            # Keep obstacles within bounds
            obstacle.pose.position.x = max(-50.0, min(50.0, obstacle.pose.position.x))
            obstacle.pose.position.y = max(-50.0, min(50.0, obstacle.pose.position.y))

    def publish_environment(self):
        marker_array = MarkerArray()
        marker_array.markers = (self.road_network + 
                                self.static_obstacles + 
                                self.dynamic_obstacles)
        
        self.update_dynamic_obstacles()
        
        self.publisher.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    environment_simulation = EnvironmentSimulation()
    rclpy.spin(environment_simulation)
    environment_simulation.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()