import rclpy
import yaml
import json
import os
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

def yaml_to_dict(path_to_yaml):
    """ Load a YAML file into a dictionary. """
    with open(path_to_yaml, "r") as f:
        return yaml.safe_load(f)

def setup_cameras(config_file_path):
    """ Resolve the correct configuration file path within a package. """
    package_path = get_package_share_directory("realsense2_camera_example")
    full_config_file_path = os.path.join(package_path, config_file_path)

    if not os.path.exists(full_config_file_path):
        raise FileNotFoundError(f"Configuration file '{full_config_file_path}' not found.")

    return yaml_to_dict(full_config_file_path)

class CameraInfoSaver(Node):
    def __init__(self):
        super().__init__('camera_info_saver')

        # Get parameters from launch file
        self.declare_parameter("config_file", "config/rs_launch_1.yaml")
        self.declare_parameter("save_dir", "/home/arg/perception-fusion/data/camera_info")

        self.config_file = self.get_parameter("config_file").value
        self.save_dir = self.get_parameter("save_dir").value

        self.get_logger().info(f"Using config file: {self.config_file}")
        self.get_logger().info(f"Saving to directory: {self.save_dir}")

        # Load cameras from YAML
        self.cameras = setup_cameras(self.config_file)

        # Use a different variable name to avoid conflicts
        self.camera_subscriptions = {}

        # Subscribe to each camera's /camera_info topic
        for camera_key, camera_values in self.cameras.items():
            camera_name = camera_values["camera_name"]
            camera_namespace = camera_values["camera_namespace"]
            topic = f"/{camera_namespace}/{camera_name}/color/camera_info"

            self.camera_subscriptions[camera_name] = self.create_subscription(
                CameraInfo,
                topic,
                lambda msg, cam=camera_name: self.camera_info_callback(msg, cam),
                10
            )
            self.get_logger().info(f"Subscribed to {topic}")

    def camera_info_callback(self, msg, camera_name):
        """
        Callback to process camera_info messages and save intrinsics to JSON.
        This function will unsubscribe after receiving the first message.
        """
        intrinsic_matrix = [
            [float(msg.k[0]), float(msg.k[1]), float(msg.k[2])],  # Convert to Python floats
            [float(msg.k[3]), float(msg.k[4]), float(msg.k[5])],
            [float(msg.k[6]), float(msg.k[7]), float(msg.k[8])]
        ]
        distortion_coeffs = [float(d) for d in msg.d]  # Convert to Python list of floats
        distortion_model = msg.distortion_model

        camera_data = {
            "camera_name": camera_name,
            "distortion_model": distortion_model,
            "camera_intrinsic": intrinsic_matrix,
            "distortion_coefficients": distortion_coeffs
        }

        # Ensure the save directory exists
        save_path = Path(self.save_dir)
        save_path.mkdir(parents=True, exist_ok=True)

        # Define the JSON file path
        json_file_path = save_path / f"{camera_name}_intrinsics.json"

        # Save to JSON file
        with open(json_file_path, "w") as json_file:
            json.dump(camera_data, json_file, indent=4)

        self.get_logger().info(f"Camera intrinsics saved to {json_file_path}")

        # Unsubscribe after first message is processed
        self.camera_subscriptions[camera_name].destroy()
        self.camera_subscriptions[camera_name] = None
        self.get_logger().info(f"Unsubscribed from /{self.cameras[camera_name]['camera_namespace']}/{camera_name}/color/camera_info")

        # If all cameras have been processed, shut down
        if all(sub is None for sub in self.camera_subscriptions.values()):
            self.get_logger().info("All cameras processed. Shutting down node.")
            rclpy.shutdown()

def main():
    rclpy.init()
    node = CameraInfoSaver()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
