from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch.actions import TimerAction

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
            parameters=[{'robot_description': Command(['xacro ', 'test.urdf'])}],
            output='screen',
        ),
    TimerAction(
        period=15.0,  
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=['rack_pinion_controller'],
            output='screen',
        )],
    ),
 ])
