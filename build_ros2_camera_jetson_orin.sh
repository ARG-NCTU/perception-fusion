#! /bin/bash

export ROS2_INSTALL_PATH=/opt/ros/humble/install

cd ros2_ws
colcon build --packages-select diagnostic_updater --symlink-install
colcon build --packages-select realsense2_camera_msgs --symlink-install
colcon build --packages-select realsense2_camera --symlink-install
colcon build --packages-select theora_image_transport --symlink-install
colcon build --packages-select image_transport_plugins --symlink-install
colcon build --packages-select compressed_image_transport --symlink-install
colcon build --packages-select compressed_depth_image_transport --symlink-install
colcon build --packages-select realsense2_camera_example --symlink-install
cd ..

source ${ROS2_INSTALL_PATH}/setup.bash
source ./ros2_ws/install/setup.bash