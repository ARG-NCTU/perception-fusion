#!/bin/bash

# List of bag file paths (add more as needed)
BAG_PATHS=(
    # "/media/arg/new_extension/bags/tainan-0411-bags/ros2-all-2025-04-11-14-57-16/ros2-all-2025-04-11-14-57-16_0.db3"
    "/media/arg/new_extension/bags/tainan-0411-bags/ros2-all-2025-04-11-14-57-16/ros2-all-2025-04-11-14-57-16_1.db3"
    "/media/arg/new_extension/bags/tainan-0411-bags/ros2-all-2025-04-11-14-57-16/ros2-all-2025-04-11-14-57-16_2.db3"
)

# Topics to play
TOPICS=(
    "/camera1/color/image_raw/compressed"
    "/camera2/color/image_raw/compressed"
    "/camera3/color/image_raw/compressed"
    "/camera1_fix/color/image_raw/compressed"
    "/camera2_fix/color/image_raw/compressed"
    "/camera3_fix/color/image_raw/compressed"
)

# Check if all bag files exist
for BAG in "${BAG_PATHS[@]}"; do
    if [ ! -f "$BAG" ]; then
        echo "❌ Bag file not found: $BAG"
        exit 1
    fi
done

# Play each bag file sequentially
echo "▶️ Starting sequential playback of bag files..."
for BAG in "${BAG_PATHS[@]}"; do
    echo "⏯️ Playing: $BAG"
    ros2 bag play "$BAG" --topics "${TOPICS[@]}" -r 3
done

echo "✅ All bag files have been played."
