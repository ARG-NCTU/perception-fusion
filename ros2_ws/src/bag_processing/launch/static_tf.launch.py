from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # radar -> map
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='radar_tf_pub',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'radar'],
        ),
        # velodyne -> map
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='velodyne_tf_pub',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'velodyne'],
        ),
    ])
