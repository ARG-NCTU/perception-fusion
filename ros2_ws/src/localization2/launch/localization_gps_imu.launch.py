from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='localization2',
            executable='localization_gps_imu',
            name='localization_gps_imu',
            output='screen',
            parameters=[],
            remappings=[
                ('/imu/data', '/imu/data'),
                ('/gps_7/navsat', '/gps_7/navsat'),
                ('~/odometry', '~/odometry'),
                ('~/set_origin', '~/set_origin'),
                ('~/set_imu_zero', '~/set_imu_zero'),
                ('~/imu_offset', '~/imu_offset')
            ]
        )
    ])

# Usage
# ros2 launch localization2 localization_gps_imu.launch.py
