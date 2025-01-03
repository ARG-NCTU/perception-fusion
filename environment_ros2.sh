#! /bin/bash

#################### ROS2 ####################

# If /opt/ros/humble/install exists, source it, otherwise source /opt/ros/humble/setup.bash
if [ -d "/opt/ros/humble/install" ]; then
    echo "Sourcing /opt/ros/humble/install/setup.bash"
    source /opt/ros/humble/install/setup.bash
else
    echo "Sourcing /opt/ros/humble/setup.bash"
    source /opt/ros/humble/setup.bash
fi
source ./ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=0
# export ROS_LOCALHOST_ONLY=1