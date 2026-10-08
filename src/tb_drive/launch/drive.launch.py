import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('tb_drive'), 'config', 'drive.yaml')
    return LaunchDescription([
        Node(
            package='tb_drive',
            executable='drive_node',
            name='drive_node',
            parameters=[cfg],
            output='screen',
        ),
    ])
