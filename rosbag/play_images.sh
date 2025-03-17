#!/bin/bash

BAG_DIR="bags/0226_ks_bags"

for BAG in $(ls "$BAG_DIR"/*.bag | sort); do
    echo "Playing $BAG ..."
    rosbag play "$BAG" --quiet
done
