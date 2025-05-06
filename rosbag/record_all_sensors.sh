#!/bin/bash

while true; do
    PREFIX=$(date +%Y-%m-%d-%H-%M-%S).bag
    BAGS=$HOME/perception-fusion/bags/ros1-all-$(date +%Y_%m%d_%H%M)

    if [ ! -d "$BAGS" ]; then
        mkdir -p $BAGS
    fi

    BAGS=$BAGS"/"$PREFIX
    echo "BAGS: "$BAGS

    # Record for 15 minutes (900 seconds) using the --duration flag
    rosbag record -O $BAGS --duration=900 \
        /camera1/color/image_raw/compressed \
        /camera2/color/image_raw/compressed \
        /camera3/color/image_raw/compressed \
        /camera4/color/image_raw/compressed \
        /camera1_fix/color/image_raw/compressed \
        /camera2_fix/color/image_raw/compressed \
        /camera3_fix/color/image_raw/compressed \
        /camera4_fix/color/image_raw/compressed \
        /camera_stitched/color/image_raw/compressed \
        /js/velodyne_points \
        /js/lidar_image/compressed \
        /js/real_velodyne/lidar_crop \
        /js/scan \
        /ais/polygon \
        /gps_43/cogsog \
        /gps_43/navsat \
        /gps_7/cogsog \
        /gps_7/navsat \
        /halo_radar/cropped_pointcloud \
        /halo_radar/cropped_scan \
        /halo_radar/merged_pointcloud \
        /halo_radar/radar_image/compressed \
        /imu/data \
        /tf \
        /tf_static 

    echo "Finished recording bag file. Starting new recording..."
done