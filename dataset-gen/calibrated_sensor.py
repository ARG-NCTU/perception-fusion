import json
import numpy as np
import os

# Define camera sensors and corresponding intrinsic file paths
camera_sensors = {
    "CAMERA_LEFT": "/home/arg/perception-fusion/ros1_ws/src/image_processing/camera_intrinsic/undistorted/js/camera1.json",
    "CAMERA_MID": "/home/arg/perception-fusion/ros1_ws/src/image_processing/camera_intrinsic/undistorted/js/camera2.json",
    "CAMERA_RIGHT": "/home/arg/perception-fusion/ros1_ws/src/image_processing/camera_intrinsic/undistorted/js/camera3.json",
    "CAMERA_BACK": "/home/arg/perception-fusion/ros1_ws/src/image_processing/camera_intrinsic/undistorted/js/camera4.json"
}

lidar_sensors = ["LIDAR_PCD", "LIDAR_IMAGE"]
radar_sensors = ["RADAR_PCD", "RADAR_IMAGE"]

# Convert yaw in degrees to quaternion (assuming rotation around Z-axis)
def quaternion_from_yaw_deg(deg):
    rad = np.deg2rad(deg)
    return [
        np.cos(rad / 2),  # w
        0.0,              # x
        0.0,              # y
        np.sin(rad / 2)   # z
    ]

output = []

# Process CAMERA sensors
for name, path in camera_sensors.items():
    if name == "CAMERA_MID":
        rotation = quaternion_from_yaw_deg(0)
    elif name == "CAMERA_LEFT":
        rotation = quaternion_from_yaw_deg(30)
    elif name == "CAMERA_RIGHT":
        rotation = quaternion_from_yaw_deg(-30)
    elif name == "CAMERA_BACK":
        rotation = quaternion_from_yaw_deg(180)

    # Load intrinsic matrix from file
    with open(path, 'r') as f:
        data = json.load(f)
        K = np.array(data['K']).reshape(3, 3).tolist()

    output.append({
        "sensor": name,
        "translation": [0.0, 0.0, 0.0],
        "rotation": rotation,
        "camera_intrinsic": [item for row in K for item in row]  # flatten to 1D
    })

# Process LIDAR sensor
for name in lidar_sensors:
    output.append({
        "sensor": name,
        "translation": [0.0, 0.0, 0.0],
        "rotation": quaternion_from_yaw_deg(0),
        "camera_intrinsic": []
    })

# Process RADAR sensor
for name in radar_sensors:
    output.append({
        "sensor": name,
        "translation": [1.0, 0.0, 0.0],
        "rotation": quaternion_from_yaw_deg(0),
        "camera_intrinsic": []
    })

# Write to JSON
save_dir = '/home/arg/perception-fusion/data/argnctu-perception/v1.0-trainval'
os.makedirs(save_dir, exist_ok=True)
with open(f"{save_dir}/calibrated_sensor.json", 'w') as f:
    json.dump(output, f, indent=4)

print(f"calibrated_sensor.json saved to {save_dir}")
