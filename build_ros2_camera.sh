#! /bin/bash

export ROS2_INSTALL_PATH=/opt/ros/humble

cd ros2_ws
colcon build --packages-select librealsense2 --cmake-args -DBUILD_EXAMPLES=OFF -DBUILD_WITH_STATIC_CRT=OFF -DBUILD_GRAPHICAL_EXAMPLES=OFF
cd ..

source ${ROS2_INSTALL_PATH}/setup.bash
source ./ros2_ws/install/setup.bash

cd ros2_ws
colcon build --packages-select realsense2_camera_msgs realsense2_description realsense2_camera
cd ..

source ${ROS2_INSTALL_PATH}/setup.bash
source ./ros2_ws/install/setup.bash