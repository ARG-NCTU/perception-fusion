from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='camera_pcd_projection',
            executable='camera_merge_node',
            name='camera_merge_node',
            output='screen',
            parameters=[
                {'left_topic': '/camera1_fix/pcd_projection/compressed'},
                {'mid_topic': '/camera2_fix/pcd_projection/compressed'},
                {'right_topic': '/camera3_fix/pcd_projection/compressed'},
                {'back_topic': '/camera4_fix/pcd_projection/compressed'},
                {'output_topic': '/camera_merged/compressed'}
            ]
        )
    ])
