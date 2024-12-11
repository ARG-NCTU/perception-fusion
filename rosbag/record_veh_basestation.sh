#! /bin/bash

PREFIX=$(date +%Y-%m-%d-%H-%M-%S)

BAGS=$HOME/perception-fusion/bags/$(date +%Y_%m%d_%H%M)

if [ ! -d "$BAGS" ]; then
    mkdir -p $BAGS
fi

BAGS=$BAGS"/"$PREFIX
echo "BAGS: "$BAGS

rosbag record -O $BAGS \
    /unity_joy_control/joy \
    /wamv/localization_pose/pose_initialize \
    /wamv/mavros/global_position/global \
    /wamv/mavros/global_position/local \
    /wamv/localization_pose/real_pose \
    /wamv/joy_combine_offsets/final_pose \
    /viewer/mavros/global_position/global \
    /viewer/mavros/global_position/local \
    /viewer/localization_pose/real_pose \
    /viewer/joy_combine_offsets/final_pose \
    /moos/wamv/ros/polygon \
    /moos/wamv/ros/pose \
    /moos/wamv/ros/track_pt \
    /moos/wamv/ros/trail \
    /pub2moos/wamv/depth \
    /pub2moos/wamv/heading \
    /pub2moos/wamv/x \
    /pub2moos/wamv/y \
