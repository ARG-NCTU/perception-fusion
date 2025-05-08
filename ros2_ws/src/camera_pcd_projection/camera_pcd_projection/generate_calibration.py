import json
import numpy as np
import os
import rclpy
from rclpy.node import Node
import ament_index_python.packages as ament_index

class CalibrationGenerator(Node):
    def __init__(self):
        super().__init__('generate_calibration')

        camera_sensors = {
            "CAMERA_LEFT": "/home/arg/perception-fusion/ros1_ws/src/image_processing/camera_intrinsic/undistorted/js/camera1.json",
            "CAMERA_MID": "/home/arg/perception-fusion/ros1_ws/src/image_processing/camera_intrinsic/undistorted/js/camera2.json",
            "CAMERA_RIGHT": "/home/arg/perception-fusion/ros1_ws/src/image_processing/camera_intrinsic/undistorted/js/camera3.json",
            "CAMERA_BACK": "/home/arg/perception-fusion/ros1_ws/src/image_processing/camera_intrinsic/undistorted/js/camera4.json"
        }

        lidar_sensors = ["LIDAR_PCD", "LIDAR_IMAGE"]
        radar_sensors = ["RADAR_PCD", "RADAR_IMAGE"]

        def quaternion_from_yaw_deg(deg):
            rad = np.deg2rad(deg)
            return [
                np.cos(rad / 2),  # w
                0.0,              # x
                0.0,              # y
                np.sin(rad / 2)   # z
            ]

        output = []

        for name, path in camera_sensors.items():
            if name == "CAMERA_MID":
                rotation = quaternion_from_yaw_deg(0)
            elif name == "CAMERA_LEFT":
                rotation = quaternion_from_yaw_deg(30)
            elif name == "CAMERA_RIGHT":
                rotation = quaternion_from_yaw_deg(-30)
            elif name == "CAMERA_BACK":
                rotation = quaternion_from_yaw_deg(180)

            with open(path, 'r') as f:
                data = json.load(f)
                K = np.array(data['new_K']).reshape(3, 3).tolist()

            output.append({
                "sensor": name,
                "translation": [0.0, 0.0, 0.0],
                "rotation": rotation,
                "camera_intrinsic": [item for row in K for item in row]
            })

        for name in lidar_sensors:
            output.append({
                "sensor": name,
                "translation": [0.0, 0.0, 0.0],
                "rotation": quaternion_from_yaw_deg(0),
                "camera_intrinsic": []
            })

        for name in radar_sensors:
            output.append({
                "sensor": name,
                "translation": [1.0, 0.0, 0.0],
                "rotation": quaternion_from_yaw_deg(0),
                "camera_intrinsic": []
            })

        # package_share = ament_index.get_package_share_directory('camera_pcd_projection')
        # save_dir = os.path.join(package_share, 'config')
        save_dir = "/home/arg/perception-fusion/ros2_ws/src/camera_pcd_projection/camera_pcd_projection/config"
        os.makedirs(save_dir, exist_ok=True)
        output_path = os.path.join(save_dir, 'calibrated_sensor.json')

        with open(output_path, 'w') as f:
            json.dump(output, f, indent=4)

        self.get_logger().info(f"calibrated_sensor.json saved to {output_path}")


def main(args=None):
    rclpy.init(args=args)
    node = CalibrationGenerator()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
