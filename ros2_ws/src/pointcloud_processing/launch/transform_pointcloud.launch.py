from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pointcloud_processing',
            executable='transform_pointcloud',
            name='transform_pointcloud',
            output='screen',
            parameters=[
                {'sub_radar_topic': '/halo_radar/cropped_pointcloud'},
                {'pub_radar_topic': '/halo_radar/transformed_pointcloud'},
                {'parent_frame_id': 'map'},
                {'child_frame_id': 'base_link'},
            ]
        )
    ])

# ros2 launch pointcloud_processing transform_pointcloud.launch.py