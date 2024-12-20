from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pointcloud_processing',
            executable='dynamic_broadcaster',
            name='dynamic_broadcaster',
            output='screen',
            parameters=[
                {'static_frame_id': 'map'},
                {'dynamic_pose': '/wamv/localization/pose'},
                {'parent_frame_id': 'map'},
                {'child_frame_id': 'base_link'}
            ]
        )
    ])

# ros2 launch pointcloud_processing dynamic_broadcaster.launch.py 

