from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pointcloud_processing',
            executable='pointcloud_to_image',
            name='pointcloud_to_image',
            output='screen',
            parameters=[
                {'pointcloud_topic': '/halo_radar/transformed_pointcloud'},
                {'image_topic': '/halo_radar/radar_image/compressed'},
                {'save_images': True},
                {'save_directory': '/home/arg/perception-fusion/ros2_ws/src/pointcloud_processing/data/radar_images'},
                {'range': 120.0},
                {'use_grayscale': False},
                {'circle_radius': 3}
            ]
        ),
    ])

# ros2 launch pointcloud_processing pointcloud_to_image.launch.py

'''
ros2 launch pointcloud_processing pointcloud_to_image.launch.py \
  --ros-args \
  -p pointcloud_topic:=/halo_radar/cropped_pointcloud \
  -p image_topic:=/halo_radar/radar_image/compressed \
  -p save_images:=True \
  -p save_directory:=/home/arg/perception-fusion/ros2_ws/src/pointcloud_processing/data/radar_images \
  -p range:=120.0 \
  -p use_grayscale:=False \
  -p circle_radius:=3
'''

