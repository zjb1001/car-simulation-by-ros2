import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import Pose

class GazeboIntegrationNode(Node):
    def __init__(self):
        super().__init__('gazebo_integration_node')
        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        
    def spawn_car_model(self, model_xml, initial_pose):
        request = SpawnEntity.Request()
        request.xml = model_xml
        request.initial_pose = initial_pose
        request.reference_frame = "world"
        
        future = self.spawn_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            self.get_logger().info(f'Car model spawned successfully: {future.result().status_message}')
        else:
            self.get_logger().error('Failed to spawn car model')

def main(args=None):
    rclpy.init(args=args)
    node = GazeboIntegrationNode()
    
    # Example usage
    car_model_xml = """
    <?xml version="1.0"?>
    <sdf version="1.6">
      <model name="car">
        <static>false</static>
        <link name="chassis">
          <pose>0 0 0.5 0 0 0</pose>
          <collision name="collision">
            <geometry>
              <box>
                <size>2.0 1.0 0.5</size>
              </box>
            </geometry>
          </collision>
          <visual name="visual">
            <geometry>
              <box>
                <size>2.0 1.0 0.5</size>
              </box>
            </geometry>
          </visual>
        </link>
      </model>
    </sdf>
    """
    
    initial_pose = Pose()
    initial_pose.position.x = 0.0
    initial_pose.position.y = 0.0
    initial_pose.position.z = 0.5
    
    node.spawn_car_model(car_model_xml, initial_pose)
    
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()