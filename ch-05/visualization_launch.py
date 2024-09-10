from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ch-05',
            executable='gazebo_integration.py',
            name='gazebo_integration_node',
            output='screen',
        ),
        Node(
            package='ch-05',
            executable='rviz_integration.py',
            name='rviz_integration_node',
            output='screen',
        ),
    ])