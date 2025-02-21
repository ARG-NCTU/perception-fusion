#!/bin/bash

# Set the base directory for bag files
BAGS_DIR="$HOME/perception-fusion/bags"
TIMESTAMP=$(date +%Y_%m%d_%H%M)
BAG_PATH="${BAGS_DIR}/radar-camera-${TIMESTAMP}"

# Create the directory if it doesn't exist
mkdir -p "$BAGS_DIR"

echo "Recording bag to: $BAG_PATH"

# Record the bag
ros2 bag record --output "$BAG_PATH" \
    /sync/halo_radar/cropped_pointcloud \
    /sync/halo_radar/cropped_scan \
    /sync/camera1/color/image_raw/compressed \
    /sync/camera2/color/image_raw/compressed \
    /sync/camera3/color/image_raw/compressed
