from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pointcloud_processing',
            executable='laserscan_qos',
            name='laserscan_qos',
            output='screen',
            parameters=[
                {'sub_laserscan_topic', '/halo_radar/cropped_scan'},
                {'pub_laserscan_topic', '/halo_radar/republished_scan'}
            ]
        ),
    ])

# ros2 launch pointcloud_processing laserscan_qos.launch.py