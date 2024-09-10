from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='car_simulation',
            executable='advanced_car_model.py',
            name='car_model'
        ),
        Node(
            package='car_simulation',
            executable='lidar_sensor.py',
            name='lidar_sensor'
        ),
        Node(
            package='car_simulation',
            executable='camera_sensor.py',
            name='camera_sensor'
        ),
        Node(
            package='car_simulation',
            executable='gps_sensor.py',
            name='gps_sensor'
        ),
        Node(
            package='car_simulation',
            executable='imu_sensor.py',
            name='imu_sensor'
        ),
        Node(
            package='car_simulation',
            executable='environment_simulation.py',
            name='environment_simulation'
        ),
        Node(
            package='car_simulation',
            executable='simulation_logger.py',
            name='simulation_logger'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', '/path/to/your/config.rviz']
        )
    ])