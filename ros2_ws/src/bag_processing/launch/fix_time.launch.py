from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='bag_processing',
            executable='fix_time',
            name='fix_time_node',
            output='screen',
            parameters=[
                {
                    'topics': [
                        '/ais/polygon',
                        '/gps_7/navsat',
                        '/imu/data',
                        '/js/velodyne_points',
                        '/halo_radar/cropped_pointcloud',
                        '/halo_radar/merged_pointcloud',
                    ]
                }
            ]
        ),
    ])
