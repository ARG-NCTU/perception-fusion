# BAG_PATH="/media/arg/new_extension/bags/tainan-0411-bags/ros2-all-2025-04-11-14-17-53/ros2-all-2025-04-11-14-17-53_0.db3"
# BAG_PATH="/home/arg/perception-fusion/bags/20250502-124545_going_jetsea_part0/20250502-124545_going_jetsea_part0_0.db3"
BAG_PATH="/home/arg/perception-fusion/bags/recorded_rosbag_going_jetsea_20250502-125945/20250502-125945_going_jetsea_part0/20250502-125945_going_jetsea_part0_0.db3"

# Check if the bag file exists
if [ ! -f "$BAG_PATH" ]; then
    echo "Bag file not found: $BAG_PATH"
    exit 1
fi

TOPICS=(
    "/ais/polygon"
    "/camera1/color/image_raw/compressed"
    "/camera2/color/image_raw/compressed"
    "/camera3/color/image_raw/compressed"
    "/camera4/color/image_raw/compressed"
    "/gps_7/navsat"
    "/halo_radar/cropped_pointcloud"
    "/imu/data"
    "/js/velodyne_points"
    "/tf"
    "/tf_static"
)

# Play the bag file with the specified topics
ros2 bag play "$BAG_PATH" --topics "${TOPICS[@]} --clock"
