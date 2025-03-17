#! /bin/bash

PREFIX=$(date +%Y-%m-%d-%H-%M-%S).bag

BAGS=$HOME/perception-fusion/bags/stitched-images-$(date +%Y_%m%d_%H%M)

if [ ! -d "$BAGS" ]; then
    mkdir -p $BAGS
fi

BAGS=$BAGS"/"$PREFIX
echo "BAGS: "$BAGS

rosbag record -O $BAGS \
    /camera_stitched/color/image_raw/compressed
