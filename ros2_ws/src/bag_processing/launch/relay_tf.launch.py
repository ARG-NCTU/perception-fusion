from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='bag_processing',  # ← 替換為你自己的 package 名稱
            executable='relay_tf',   # ← 你在 setup.py 中定義的 entry point name
            name='relay_tf',
            output='screen'
        )
    ])
