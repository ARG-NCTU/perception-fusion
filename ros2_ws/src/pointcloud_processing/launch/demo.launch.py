from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        # LaserScan qos
        Node(
            package='pointcloud_processing',
            executable='laserscan_qos',
            name='laserscan_qos',
            output='screen',
            parameters=[
                {'sub_laserscan_topic': '/halo_radar/cropped_scan'},
                {'pub_laserscan_topic': '/halo_radar/republished_scan'}
            ]
        ),

        # PointCloud2 to Image Node
        Node(
            package='pointcloud_processing',
            executable='pointcloud_to_image',
            name='pointcloud_to_image',
            output='screen',
            parameters=[
                {'pointcloud_topic': '/halo_radar/cropped_pointcloud'},
                {'image_topic': '/halo_radar/radar_image/compressed'},
                {'save_images': False},
                {'save_directory': '/home/arg/perception-fusion/ros2_ws/src/pointcloud_processing/data/radar_images'},
                {'range': 120.0},
                {'use_grayscale': False},
                {'circle_radius': 3}
            ]
        )
    ])

# ros2 launch pointcloud_processing demo.launch.py