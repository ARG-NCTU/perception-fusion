#! /bin/bash

#################### ROS1 ####################

source environment_ros1.sh

#################### ROS2-ROS1-BRIDGE ####################

source /opt/ros/foxy/setup.bash
source ./ros2_ros1_bridge_ws/install/setup.bash
export ROS_DOMAIN_ID=0
# export ROS_LOCALHOST_ONLY=1