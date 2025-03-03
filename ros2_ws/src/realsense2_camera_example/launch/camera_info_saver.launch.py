from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    declare_config_file = DeclareLaunchArgument(
        "config_file",
        default_value="config/rs_launch_1.yaml",
        description="Path to the YAML configuration file",
    )

    declare_save_dir = DeclareLaunchArgument(
        "save_dir",
        default_value="/home/arg/perception-fusion/data/camera_info",
        description="Directory to save camera intrinsic parameters",
    )

    return LaunchDescription([
        declare_config_file,
        declare_save_dir,
        Node(
            package="realsense2_camera_example",
            executable="camera_info_saver",
            name="camera_info_saver",
            output="screen",
            parameters=[
                {"config_file": LaunchConfiguration("config_file")},
                {"save_dir": LaunchConfiguration("save_dir")}
            ],
        ),
    ])
