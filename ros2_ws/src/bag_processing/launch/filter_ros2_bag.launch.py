from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='bag_processing',
            executable='filter_ros2_bag',
            name='filter_ros2_bag_node_1',
            parameters=[{
                'input_bag': '/media/arg/new_extension/bags/tainan-0411-bags/ros2-all-2025-04-11-12-19-04',
                'output_bag': 'bags/ros2-gps-imu-10-11-knots-2025-04-11-12-21',
                'start_time': '2025-04-11T12:21:00+08:00',
                'end_time': '2025-04-11T12:22:59+08:00',
                'topics': [
                    '/gps_43/cogsog',
                    '/gps_43/navsat',
                    '/gps_7/cogsog',
                    '/gps_7/navsat',
                    '/pixhawk1/mavros/global_position/raw/fix',
                    '/pixhawk1/mavros/imu/data',
                    '/pixhawk2/mavros/global_position/raw/fix',
                    '/pixhawk2/mavros/imu/data',
                ]
            }]
        ),
        Node(
            package='bag_processing',
            executable='filter_ros2_bag',
            name='filter_ros2_bag_node_2',
            parameters=[{
                'input_bag': '/media/arg/new_extension/bags/tainan-0411-bags/ros2-all-2025-04-11-12-19-04',
                'output_bag': 'bags/ros2-gps-imu-15-18-knots-2025-04-11-12-23',
                'start_time': '2025-04-11T12:23:00+08:00',
                'end_time': '2025-04-11T12:23:59+08:00',
                'topics': [
                    '/gps_43/cogsog',
                    '/gps_43/navsat',
                    '/gps_7/cogsog',
                    '/gps_7/navsat',
                    '/pixhawk1/mavros/global_position/raw/fix',
                    '/pixhawk1/mavros/imu/data',
                    '/pixhawk2/mavros/global_position/raw/fix',
                    '/pixhawk2/mavros/imu/data',
                ]
            }]
        ),
        Node(
            package='bag_processing',
            executable='filter_ros2_bag',
            name='filter_ros2_bag_node_3',
            parameters=[{
                'input_bag': '/media/arg/new_extension/bags/tainan-0411-bags/ros2-all-2025-04-11-12-19-04',
                'output_bag': 'bags/ros2-gps-imu-15-16-knots-2025-04-11-12-24',
                'start_time': '2025-04-11T12:24:00+08:00',
                'end_time': '2025-04-11T12:26:29+08:00',
                'topics': [
                    '/gps_43/cogsog',
                    '/gps_43/navsat',
                    '/gps_7/cogsog',
                    '/gps_7/navsat',
                    '/pixhawk1/mavros/global_position/raw/fix',
                    '/pixhawk1/mavros/imu/data',
                    '/pixhawk2/mavros/global_position/raw/fix',
                    '/pixhawk2/mavros/imu/data',
                ]
            }]
        ),
        Node(
            package='bag_processing',
            executable='filter_ros2_bag',
            name='filter_ros2_bag_node_4',
            parameters=[{
                'input_bag': '/media/arg/new_extension/bags/tainan-0411-bags/ros2-all-2025-04-11-12-19-04',
                'output_bag': 'bags/ros2-gps-imu-20-knots-2025-04-11-12-26',
                'start_time': '2025-04-11T12:26:30+08:00',
                'end_time': '2025-04-11T12:28:30+08:00',
                'topics': [
                    '/gps_43/cogsog',
                    '/gps_43/navsat',
                    '/gps_7/cogsog',
                    '/gps_7/navsat',
                    '/pixhawk1/mavros/global_position/raw/fix',
                    '/pixhawk1/mavros/imu/data',
                    '/pixhawk2/mavros/global_position/raw/fix',
                    '/pixhawk2/mavros/imu/data',
                ]
            }]
        ),
        Node(
            package='bag_processing',
            executable='filter_ros2_bag',
            name='filter_ros2_bag_node_5',
            parameters=[{
                'input_bag': '/media/arg/new_extension/bags/tainan-0411-bags/ros2-all-2025-04-11-12-19-04',
                'output_bag': 'bags/ros2-gps-imu-10-knots-2025-04-11-12-32',
                'start_time': '2025-04-11T12:32:00+08:00',
                'end_time': '2025-04-11T12:34:59+08:00',
                'topics': [
                    '/gps_43/cogsog',
                    '/gps_43/navsat',
                    '/gps_7/cogsog',
                    '/gps_7/navsat',
                    '/pixhawk1/mavros/global_position/raw/fix',
                    '/pixhawk1/mavros/imu/data',
                    '/pixhawk2/mavros/global_position/raw/fix',
                    '/pixhawk2/mavros/imu/data',
                ]
            }]
        ),
        Node(
            package='bag_processing',
            executable='filter_ros2_bag',
            name='filter_ros2_bag_node_6',
            parameters=[{
                'input_bag': '/media/arg/new_extension/bags/tainan-0411-bags/ros2-all-2025-04-11-12-19-04',
                'output_bag': 'bags/ros2-gps-imu-15-knots-2025-04-11-12-35',
                'start_time': '2025-04-11T12:35:00+08:00',
                'end_time': '2025-04-11T12:37:00+08:00',
                'topics': [
                    '/gps_43/cogsog',
                    '/gps_43/navsat',
                    '/gps_7/cogsog',
                    '/gps_7/navsat',
                    '/pixhawk1/mavros/global_position/raw/fix',
                    '/pixhawk1/mavros/imu/data',
                    '/pixhawk2/mavros/global_position/raw/fix',
                    '/pixhawk2/mavros/imu/data',
                ]
            }]
        ),
        Node(
            package='bag_processing',
            executable='filter_ros2_bag',
            name='filter_ros2_bag_node_7',
            parameters=[{
                'input_bag': '/media/arg/new_extension/bags/tainan-0411-bags/ros2-all-2025-04-11-12-19-04',
                'output_bag': 'bags/ros2-gps-imu-7-8-knots-2025-04-11-12-44',
                'start_time': '2025-04-11T12:44:00+08:00',
                'end_time': '2025-04-11T12:46:00+08:00',
                'topics': [
                    '/gps_43/cogsog',
                    '/gps_43/navsat',
                    '/gps_7/cogsog',
                    '/gps_7/navsat',
                    '/pixhawk1/mavros/global_position/raw/fix',
                    '/pixhawk1/mavros/imu/data',
                    '/pixhawk2/mavros/global_position/raw/fix',
                    '/pixhawk2/mavros/imu/data',
                ]
            }]
        ),
    ])
