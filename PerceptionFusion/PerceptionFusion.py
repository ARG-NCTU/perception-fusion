import os
import numpy as np
import cv2
from glob import glob
from PerceptionFusion.dataloader import DataLoader
from PerceptionFusion.utils import project_pcl_to_image, pcd_to_camera_frame
import pandas as pd

class PerceptionFusion:
    def __init__(self, data_root):
        self.loader = DataLoader(data_root)

    def get_timestamp_from_frame(self, frame_index, sensor_dir_name):
        pcd_dir = os.path.join(self.loader.samples_dir, sensor_dir_name)
        files = sorted(glob(os.path.join(pcd_dir, "*.pcd")))
        if frame_index >= len(files):
            raise IndexError(f"Frame index {frame_index} exceeds number of available frames in {sensor_dir_name}")
        filename = os.path.basename(files[frame_index])
        parts = filename.split("_")
        for part in reversed(parts):
            if part.startswith("c"):
                timestamp = part.replace("c", "").replace(".pcd", "")
                return timestamp
        raise ValueError(f"Could not extract timestamp from {filename}")

    def visualize(self, timestamp, image_sensor, pcd_sensor, save_path=None):
        image = self.loader.load_camera_image(image_sensor, timestamp)
        pcd = self.loader.load_pointcloud(pcd_sensor, timestamp)

        if image is None or pcd is None:
            print(f"[Error] Missing data for {timestamp} in {image_sensor} or {pcd_sensor}")
            return

        intrinsic = self.loader.get_intrinsic(image_sensor)
        cam_ext = self.loader.get_extrinsic(image_sensor)
        pcd_ext = self.loader.get_extrinsic(pcd_sensor)
        extrinsic = cam_ext @ np.linalg.inv(pcd_ext)

        pcd_np = np.asarray(pcd.points)
        print(f"[Info] {pcd_sensor} points: {pcd_np.shape[0]}")
        print(f"points: {pcd_np[:5]}")
        pcl_df = pd.DataFrame(pcd_np, columns=['x','y','z'])
        pcl_df['rcs'] = 1.0

        # Apply coordinate system conversion before projection
        pcl_df[['x','y','z']] = pcd_to_camera_frame(pcl_df[['x','y','z']].values)

        uvs, _, _ = project_pcl_to_image(pcl_df, extrinsic, intrinsic, image.shape)
        for pt in uvs:
            print(f"pt: {pt}")
            cv2.circle(image, pt, 2, (0, 255, 0), -1)

        if save_path:
            os.makedirs(os.path.dirname(save_path), exist_ok=True)
            cv2.imwrite(save_path, image)
            print(f"[Saved] {save_path}")

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="Perception Fusion Visualization")
    parser.add_argument("--frame", type=int, required=True, help="Frame index")
    parser.add_argument("--proj", choices=["lidar", "radar", "both"], default="both", help="Which pointcloud to project")
    parser.add_argument("--data", type=str, default="/home/arg/perception-fusion/data/argnctu-perception", help="Path to the dataset")
    parser.add_argument("--save_dir", type=str, default="visualization", help="Base directory to save images")
    args = parser.parse_args()

    dataroot = args.data
    if not os.path.exists(dataroot):
        raise FileNotFoundError(f"Data root {dataroot} does not exist")
    fusion = PerceptionFusion(dataroot)

    if args.proj == "lidar":
        timestamp = fusion.get_timestamp_from_frame(args.frame, "LIDAR_PCD")
    else:
        timestamp = fusion.get_timestamp_from_frame(args.frame, "RADAR_PCD")

    camera_list = ["CAMERA_LEFT", "CAMERA_MID", "CAMERA_RIGHT", "CAMERA_BACK"]
    frame_dir = os.path.join(args.save_dir, str(args.frame))

    for cam in camera_list:
        if args.proj in ["radar", "both"]:
            fusion.visualize(timestamp, cam, "RADAR_PCD", save_path=os.path.join(frame_dir, f"{cam}_radar.png"))
        if args.proj in ["lidar", "both"]:
            fusion.visualize(timestamp, cam, "LIDAR_PCD", save_path=os.path.join(frame_dir, f"{cam}_lidar.png"))

# Usage:
# cd ~/perception-fusion
# python3 -m PerceptionFusion.PerceptionFusion --frame 10 --proj lidar --data /home/arg/perception-fusion/data/argnctu-perception --save_dir visualization
