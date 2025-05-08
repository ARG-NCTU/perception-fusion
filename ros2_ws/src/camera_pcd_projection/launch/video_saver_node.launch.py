from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='camera_pcd_projection',
            executable='video_saver_node',
            name='video_saver_node',
            output='screen',
            parameters=[
                {'camera_topic': '/camera_merged/compressed'},
                {'save_dir': '/home/arg/perception-fusion/data/videos'}
            ]
        )
    ])
