#!/bin/bash

BAG_DIR="/media/arg/new_extension/bags/0226_radarbags"

for BAG_SUB_DIR in $(ls -d $BAG_DIR/*); do
    echo "Subdirectory: $BAG_SUB_DIR"

    for BAG_FILE in $(ls $BAG_SUB_DIR/*.bag); do
        echo "Processing bag file: $BAG_FILE"
        rosbag play $BAG_FILE
    done
done

