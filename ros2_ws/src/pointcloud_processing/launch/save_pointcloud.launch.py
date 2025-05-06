from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pointcloud_processing',
            executable='save_pointcloud',
            name='save_radar_pointcloud',
            output='screen',
            parameters=[
                {'pointcloud_topic': '/halo_radar/cropped_pointcloud'},
                {'save_directory': '/home/arg/perception-fusion/ros2_ws/src/pointcloud_processing/data/radar_pcds'}
            ]
        ),
        Node(
            package='pointcloud_processing',
            executable='save_pointcloud',
            name='save_lidar_pointcloud',
            output='screen',
            parameters=[
                {'pointcloud_topic': '/js/velodyne_points'},
                {'save_directory': '/home/arg/perception-fusion/ros2_ws/src/pointcloud_processing/data/lidar_pcds'}
            ]
        )
    ])

# ros2 launch pointcloud_processing save_pointcloud.launch.py