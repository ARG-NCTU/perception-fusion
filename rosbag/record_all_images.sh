#! /bin/bash

PREFIX=$(date +%Y-%m-%d-%H-%M-%S).bag

BAGS=$HOME/perception-fusion/bags/all-images-$(date +%Y_%m%d_%H%M)

if [ ! -d "$BAGS" ]; then
    mkdir -p $BAGS
fi

BAGS=$BAGS"/"$PREFIX
echo "BAGS: "$BAGS

rosbag record -O $BAGS \
    /camera1/color/image_raw/compressed \
    /camera2/color/image_raw/compressed \
    /camera3/color/image_raw/compressed \
    /camera_stitched/color/image_raw/compressed \
    /detection_result_img/camera_stitched/compressed
