from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os

def generate_launch_description():
    return LaunchDescription([
        # Node(
        #     package='realsense2_camera',
        #     executable='rs_launch.py',
        #     name='realsense_camera',
        #     output='screen'
        # ),
        Node(
            package='vins',
            namespace='process',
            executable='process.py',
            name='proces'
        ),
        Node(
            package='vins',
            namespace='vins_transfer',
            executable='vins_transfer.py',
            name='vins_transfer',
            output='screen'
        )
    ])
