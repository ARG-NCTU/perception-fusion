#! /bin/bash

PREFIX=$(date +%Y-%m-%d-%H-%M-%S)

BAGS=$HOME/perception-fusion/bags/images-$(date +%Y_%m%d_%H%M)

if [ ! -d "$BAGS" ]; then
    mkdir -p $BAGS
fi

BAGS=$BAGS"/"$PREFIX
echo "BAGS: "$BAGS

rosbag record -O $BAGS \
    /camera_left/color/image_raw/compressed \
    /detection_result_img/camera_left/compressed \
    /camera_middle/color/image_raw/compressed \
    /detection_result_img/camera_middle/compressed \
    /camera_right/color/image_raw/compressed \
    /detection_result_img/camera_right/compressed 
