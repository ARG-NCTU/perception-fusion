# BAG_PATH="/media/arg/new_extension/bags/tainan-0411-bags/ros2-all-2025-04-11-14-17-53/ros2-all-2025-04-11-14-17-53_0.db3"
BAG_PATH="/media/arg/new_extension2/bags/tainan-0411-bags/ros2-all-2025-04-11-14-57-16/ros2-all-2025-04-11-14-57-16_0.db3"

# Check if the bag file exists
if [ ! -f "$BAG_PATH" ]; then
    echo "Bag file not found: $BAG_PATH"
    exit 1
fi

TOPICS=(
    "/ais/polygon"
    "/camera1/color/image_raw/compressed"
    "/camera1_fix/color/image_raw/compressed"
    "/camera2/color/image_raw/compressed"
    "/camera2_fix/color/image_raw/compressed"
    "/camera3/color/image_raw/compressed"
    "/camera3_fix/color/image_raw/compressed"
    "/camera4/color/image_raw/compressed"
    "/gps_7/navsat"
    "/halo_radar/cropped_pointcloud"
    "/halo_radar/cropped_scan"
    "/halo_radar/merged_pointcloud"
    "/imu/data"
    "/js/real_velodyne/lidar_crop"
    "/js/scan"
    "/js/velodyne_points"
)

# Play the bag file with the specified topics
ros2 bag play "$BAG_PATH" --topics "${TOPICS[@]}"
