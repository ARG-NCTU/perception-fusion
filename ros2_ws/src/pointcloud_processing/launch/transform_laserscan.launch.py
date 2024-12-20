from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pointcloud_processing',
            executable='transform_laserscan',
            name='transform_laserscan',
            output='screen',
            parameters=[
                {'sub_laserscan_topic': '/halo_radar/republished_scan'},
                {'pub_laserscan_topic': '/halo_radar/transformed_scan'},
                {'parent_frame_id': 'map'},
                {'child_frame_id': 'base_link'}
            ]
        ),
    ])


# ros2 launch pointcloud_processing transform_laserscan.launch.py