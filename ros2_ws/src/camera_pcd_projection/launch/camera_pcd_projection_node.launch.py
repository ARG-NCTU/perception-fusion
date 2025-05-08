from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

# Path to the calibrated_sensor.json
calib_path = "/home/arg/perception-fusion/ros2_ws/src/camera_pcd_projection/camera_pcd_projection/config/calibrated_sensor.json"

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='camera_pcd_projection',
            executable='camera_pcd_projection_node',
            name='camera1_projection_left',
            parameters=[
                {'camera_position': 'CAMERA_LEFT'},
                {'pcd_sensor': 'LIDAR_PCD'},
                {'camera_topic': '/camera1_fix/color/image_raw/compressed'},
                {'pointcloud_topic': '/js/velodyne_points'},
                {'calibrated_path': calib_path},
                {'output_image_topic': '/camera1_fix/pcd_projection/compressed'}
            ]
        ),
        Node(
            package='camera_pcd_projection',
            executable='camera_pcd_projection_node',
            name='camera2_projection_mid',
            parameters=[
                {'camera_position': 'CAMERA_MID'},
                {'pcd_sensor': 'LIDAR_PCD'},
                {'camera_topic': '/camera2_fix/color/image_raw/compressed'},
                {'pointcloud_topic': '/js/velodyne_points'},
                {'calibrated_path': calib_path},
                {'output_image_topic': '/camera2_fix/pcd_projection/compressed'}
            ]
        ),
        Node(
            package='camera_pcd_projection',
            executable='camera_pcd_projection_node',
            name='camera3_projection_right',
            parameters=[
                {'camera_position': 'CAMERA_RIGHT'},
                {'pcd_sensor': 'LIDAR_PCD'},
                {'camera_topic': '/camera3_fix/color/image_raw/compressed'},
                {'pointcloud_topic': '/js/velodyne_points'},
                {'calibrated_path': calib_path},
                {'output_image_topic': '/camera3_fix/pcd_projection/compressed'}
            ]
        ),
        Node(
            package='camera_pcd_projection',
            executable='camera_pcd_projection_node',
            name='camera4_projection_back',
            parameters=[
                {'camera_position': 'CAMERA_BACK'},
                {'pcd_sensor': 'LIDAR_PCD'},
                {'camera_topic': '/camera4_fix/color/image_raw/compressed'},
                {'pointcloud_topic': '/js/velodyne_points'},
                {'calibrated_path': calib_path},
                {'output_image_topic': '/camera4_fix/pcd_projection/compressed'}
            ]
        )
    ])
