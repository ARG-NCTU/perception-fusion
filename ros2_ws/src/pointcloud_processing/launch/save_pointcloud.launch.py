from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pointcloud_processing',
            executable='save_pointcloud',
            name='save_pointcloud',
            output='screen',
            parameters=[
                {'pointcloud_topic': '/halo_radar/cropped_pointcloud'},
                {'parent_frame_id': 'radar_global_map'},
                {'child_frame_id': 'radar'},
                {'save_directory': '/home/arg/perception-fusion/ros2_ws/src/pointcloud_processing/data/radar_pcds'}
            ]
        )
    ])

# ros2 launch pointcloud_processing save_pointcloud.launch.py
    
'''
ros2 launch pointcloud_processing save_pointcloud.launch.py \
    --ros-args \
    -p pointcloud_topic:=/halo_radar/cropped_pointcloud \
    -p parent_frame_id:=radar_global_map \
    -p child_frame_id:=radar \
    -p save_directory:=/home/arg/perception-fusion/ros2_ws/src/pointcloud_processing/data/radar_pcds
'''