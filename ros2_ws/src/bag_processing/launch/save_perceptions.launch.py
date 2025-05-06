from launch import LaunchDescription
from launch_ros.actions import Node
import json

def generate_launch_description():
    topics = {
        'camera_left': '/camera1_fix/color/image_raw/compressed',
        'camera_mid': '/camera2_fix/color/image_raw/compressed',
        'camera_right': '/camera3_fix/color/image_raw/compressed',
        'camera_stitched': '/camera_stitched/color/image_raw/compressed',
        'camera_back': '/camera4_fix/color/image_raw/compressed',
        'radar_image': '/halo_radar/radar_image/compressed',
        'radar_pcd': '/halo_radar/cropped_pointcloud',
        'lidar_image': '/js/lidar_image/compressed',
        'lidar_pcd': '/js/velodyne_points'
    }

    return LaunchDescription([
        Node(
            package='bag_processing',
            executable='save_perceptions',
            name='save_perceptions',
            output='screen',
            parameters=[
                {'save_base_directory': '/home/arg/perception-fusion/data/argnctu-perception/samples'},
                {'topics': json.dumps(topics)}
            ]
        )
    ])
