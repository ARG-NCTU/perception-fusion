from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='image_processing',
            executable='camera_node',
            name='camera1',
            parameters=[
                {'node_name': 'camera1_rtsp_to_compressed_image'},
                {'rtsp_url': 'rtsp://admin:admin@192.168.0.101:554/video'},
                {'topic_name': '/camera1/color/image_raw/compressed'}
            ]
        ),
        Node(
            package='image_processing',
            executable='camera_node',
            name='camera2',
            parameters=[
                {'node_name': 'camera2_rtsp_to_compressed_image'},
                {'rtsp_url': 'rtsp://admin:admin@192.168.0.102:554/video'},
                {'topic_name': '/camera2/color/image_raw/compressed'}
            ]
        ),
        Node(
            package='image_processing',
            executable='camera_node',
            name='camera3',
            parameters=[
                {'node_name': 'camera3_rtsp_to_compressed_image'},
                {'rtsp_url': 'rtsp://admin:admin@192.168.0.103:554/video'},
                {'topic_name': '/camera3/color/image_raw/compressed'}
            ]
        )
    ])
