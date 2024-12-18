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
                {'pointcloud_topic': '/halo_radar/cropped_pointcloud'},
                {'image_topic': '/halo_radar/radar_image/compressed'},
                {'parent_frame_id': 'radar_global_map'},
                {'child_frame_id': 'radar'},
                {'save_images': True},
                {'save_directory': '/home/arg/perception-fusion/ros2_ws/src/pointcloud_processing/data/radar_images'}
            ]
        )
    ])

# ros2 launch pointcloud_processing pointcloud_to_image.launch.py

'''
ros2 launch pointcloud_processing pointcloud_to_image.launch.py \
  --ros-args \
  -p pointcloud_topic:=/halo_radar/cropped_pointcloud \
  -p image_topic:=/halo_radar/radar_image/compressed \
  -p parent_frame_id:=radar_global_map \
  -p child_frame_id:=radar \
  -p save_images:=True \
  -p save_directory:=/home/arg/perception-fusion/ros2_ws/src/pointcloud_processing/data/radar_images
'''

