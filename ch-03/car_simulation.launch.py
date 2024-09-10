from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='car_simulation',
            executable='car_model_node',
            name='car_model'
        ),
        Node(
            package='car_simulation',
            executable='sensor_simulation_node',
            name='sensor_simulation'
        ),
        Node(
            package='car_simulation',
            executable='environment_simulation_node',
            name='environment_simulation'
        ),
        Node(
            package='car_simulation',
            executable='visualization_bridge_node',
            name='visualization_bridge'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', 'path/to/your/config.rviz']
        )
    ])