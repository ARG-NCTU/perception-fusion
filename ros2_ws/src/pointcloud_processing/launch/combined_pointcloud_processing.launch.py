from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Fake Pose Publisher
        Node(
            package='pointcloud_processing',
            executable='fake_pose_publisher',
            name='fake_pose_publisher',
            output='screen',
            parameters=[
                {'pose_pub_topic': '/wamv/localization/pose'}
            ]
        ),

        # Dynamic Broadcaster
        Node(
            package='pointcloud_processing',
            executable='dynamic_broadcaster',
            name='dynamic_broadcaster',
            output='screen',
            parameters=[
                {'static_frame_id': 'map'},
                {'dynamic_pose': '/wamv/localization/pose'},
                {'parent_frame_id': 'map'},
                {'child_frame_id': 'base_link'}
            ]
        ),

        # Transform PointCloud Node
        Node(
            package='pointcloud_processing',
            executable='transform_pointcloud',
            name='transform_pointcloud',
            output='screen',
            parameters=[
                {'sub_radar_topic': '/halo_radar/cropped_pointcloud'},
                {'pub_radar_topic': '/halo_radar/transformed_pointcloud'},
                {'parent_frame_id': 'map'},
                {'child_frame_id': 'base_link'}
            ]
        ),

        # # PointCloud2 to Image Node
        # Node(
        #     package='pointcloud_processing',
        #     executable='pointcloud_to_image',
        #     name='pointcloud_to_image',
        #     output='screen',
        #     parameters=[
        #         {'pointcloud_topic': '/halo_radar/cropped_pointcloud'},
        #         {'image_topic': '/halo_radar/radar_image/compressed'},
                # {'parent_frame_id': 'radar_base_link'},
                # {'child_frame_id': 'radar'},
        #         {'save_images': True},
        #         {'save_directory': '/home/arg/perception-fusion/ros2_ws/src/pointcloud_processing/data/radar_images'}
        #     ]
        # ),

        # # Save PointCloud Node
        # Node(
        #     package='pointcloud_processing',
        #     executable='save_pointcloud',
        #     name='save_pointcloud',
        #     output='screen',
        #     parameters=[
        #         {'pointcloud_topic': '/halo_radar/cropped_pointcloud'},
        #         {'parent_frame_id': 'radar_base_link'},
        #         {'child_frame_id': 'radar'},
        #         {'save_directory': '/home/arg/perception-fusion/ros2_ws/src/pointcloud_processing/data/radar_pcds'}
        #     ]
        # )
    ])

# ros2 launch pointcloud_processing combined_pointcloud_processing.launch.py