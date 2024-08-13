from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='vins',
            namespace='vins_node',
            executable='vins_node',
            name='vins_node',
            arguments=['--config_file', '/home/vt008/vins_fusion_ros2_edited/src/VINS-Fusion-ROS2/config/realsense_d435i/realsense_stereo_imu_config.yaml']
        ),
        Node(
            package='vins',
            namespace='vins_transfer',
            executable='vins_transfer.py',
            name='vins_transfer',
            output='screen'
        )
    ])
