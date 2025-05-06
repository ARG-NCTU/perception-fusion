from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pointcloud_processing',
            executable='pointcloud_to_image',
            name='radar_pointcloud_to_image',
            output='screen',
            parameters=[
                {'pointcloud_topic': '/halo_radar/cropped_pointcloud'},
                {'image_topic': '/halo_radar/radar_image/compressed'},
                {'save_images': False},
                {'save_directory': '/home/arg/perception-fusion/ros2_ws/src/pointcloud_processing/data/radar_images'},
                {'add_grid': False},
                {'range': 120.0},
                {'use_grayscale': False},
                {'timer_used': False},
            ]
        ),
        Node(
            package='pointcloud_processing',
            executable='pointcloud_to_image',
            name='lidar_pointcloud_to_image',
            output='screen',
            parameters=[
                {'pointcloud_topic': '/js/velodyne_points'},
                {'image_topic': '/js/lidar_image/compressed'},
                {'save_images': False},
                {'save_directory': '/home/arg/perception-fusion/ros2_ws/src/pointcloud_processing/data/lidar_images'},
                {'add_grid': False},
                {'range': 120.0},
                {'use_grayscale': False},
                {'timer_used': False},
            ]
        ),
    ])

# ros2 launch pointcloud_processing pointcloud_to_image.launch.py

