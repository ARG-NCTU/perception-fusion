#! /bin/bash

#################### ROS1 ####################

if [ $# -gt 0 ]; then
	export ROS_MASTER_IP=$1
    echo "ROS_MASTER_IP set to $ROS_MASTER_IP"
    source set_ros1_master.sh $ROS_MASTER_IP
else
    source set_ros1_master.sh 127.0.0.1
fi

if [ $# -gt 0 ]; then
	export ROS_IP=$2
    echo "ROS_IP set to $ROS_IP"
    source set_ros1_ip.sh $ROS_IP
else
    source set_ros1_ip.sh 127.0.0.1
fi

#################### ROS2-ROS1-BRIDGE ####################

source /opt/ros/humble/setup.bash
source ./ros-humble-ros1-bridge/install/local_setup.bash
# export ROS_DOMAIN_ID=0
# export ROS_LOCALHOST_ONLY=1