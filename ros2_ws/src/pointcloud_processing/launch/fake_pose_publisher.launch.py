from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pointcloud_processing',  # Replace with your package name
            executable='fake_pose_publisher',  # Replace with your node executable name
            name='fake_pose_publisher',
            output='screen',
            parameters=[
                {'pose_pub_topic': '/wamv/localization/pose'}
            ]
        )
    ])

# ros2 launch pointcloud_processing fake_pose_publisher.launch.py

'''
ros2 launch pointcloud_processing fake_pose_publisher.launch.py \
  --ros-args -p pose_pub_topic:=/wamv/localization/pose
'''

