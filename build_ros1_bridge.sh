#! /bin/bash

export ROS1_INSTALL_PATH=/opt/ros/noetic
export ROS2_INSTALL_PATH=/opt/ros/foxy

cd ros2_ws
colcon build --symlink-install --packages-skip ros1_bridge
cd ..

source ${ROS1_INSTALL_PATH}/setup.bash
source ${ROS2_INSTALL_PATH}/setup.bash
source ./ros2_ws/install/setup.bash
source ./ros1_ws/devel/setup.bash

cd ros2_ws
colcon build --symlink-install --packages-select ros1_bridge --cmake-force-configure
cd ..