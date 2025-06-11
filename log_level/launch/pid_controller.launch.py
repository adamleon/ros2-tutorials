from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='log_level',
            executable='pid_controller_good',
            name='pid_controller_node',
            arguments=['--ros-args', '--log-level', 'info'],
            output='screen'
        )
    ])