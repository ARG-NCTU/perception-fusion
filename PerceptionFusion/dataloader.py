### dataloader.py
import os
import json
import cv2
import open3d as o3d
import numpy as np
from glob import glob
from scipy.spatial.transform import Rotation as R

class DataLoader:
    def __init__(self, root_dir):
        self.root_dir = root_dir
        self.calib_path = os.path.join(root_dir, "v1.0-trainval", "calibrated_sensor.json")
        self.samples_dir = os.path.join(root_dir, "samples")
        self.calibration = self._load_calibration()

    def _load_calibration(self):
        with open(self.calib_path, 'r') as f:
            data = json.load(f)

        calib = {}
        for item in data:
            sensor = item['sensor']
            quat = item['rotation']
            r = R.from_quat([quat[1], quat[2], quat[3], quat[0]])
            rot_mat = r.as_matrix()
            trans = np.array(item['translation']).reshape(3, 1)

            extrinsic = np.eye(4)
            extrinsic[:3, :3] = rot_mat
            extrinsic[:3, 3] = trans.flatten()

            intrinsic = None
            if item['camera_intrinsic']:
                intrinsic = np.array(item['camera_intrinsic']).reshape(3, 3)

            calib[sensor] = {
                'extrinsic': extrinsic,
                'intrinsic': intrinsic
            }

        return calib

    def load_camera_image(self, sensor, timestamp):
        img_dir = os.path.join(self.samples_dir, sensor)
        pattern = f"*{timestamp}*.png"
        files = glob(os.path.join(img_dir, pattern))
        return cv2.imread(files[0]) if files else None

    def load_pointcloud(self, sensor, timestamp):
        pcd_dir = os.path.join(self.samples_dir, sensor)
        pattern = f"*{timestamp}*.pcd"
        files = glob(os.path.join(pcd_dir, pattern))
        return o3d.io.read_point_cloud(files[0]) if files else None

    def get_extrinsic(self, sensor):
        return self.calibration[sensor]['extrinsic']

    def get_intrinsic(self, sensor):
        intr = self.calibration[sensor]['intrinsic']
        if intr is None:
            raise ValueError(f"Sensor {sensor} has no intrinsic matrix.")
        return intr
