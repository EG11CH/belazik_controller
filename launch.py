from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=['test.yaml'],
            output='screen',
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=['robot_urdf.urdf'],
            output='screen',
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['test.cpp'],
            output='screen',
        ),
        ),
    ])

