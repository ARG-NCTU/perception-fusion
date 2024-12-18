#! /bin/bash

export ROS2_INSTALL_PATH=/opt/ros/humble

cd ros2_ws
colcon build --packages-select pointcloud_processing --symlink-install
cd ..

source ${ROS2_INSTALL_PATH}/setup.bash
source ./ros2_ws/install/setup.bash