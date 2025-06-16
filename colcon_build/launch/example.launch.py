from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('colcon_build'),
        'config',
        'some_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='colcon_build',
            executable='some_node',
            name='some_node',
            parameters=[config_file]
        )
    ])