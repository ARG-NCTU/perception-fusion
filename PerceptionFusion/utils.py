import numpy as np
import matplotlib.pyplot as plt

def view_points(points, intrinsic, normalize=True):
    """
    Projects 3D points to 2D using the intrinsic camera matrix.
    :param points: (3, N) ndarray of 3D points.
    :param intrinsic: (3, 3) intrinsic camera matrix.
    :param normalize: If True, normalize the projected points.
    :return: (3, N) ndarray of projected 2D points.
    """
    assert points.shape[0] == 3
    points_hom = np.vstack((points, np.ones((1, points.shape[1]))))
    proj = intrinsic @ points_hom[:3, :]
    if normalize:
        proj[:2, :] /= proj[2, :]
    return proj

def pcd_to_camera_frame(points_pcd):
    """
    Converts points from PCD coordinate frame (X-forward, Y-left, Z-up)
    to camera frame (X-right, Y-down, Z-forward).

    :param points_pcd: (N, 3) ndarray
    :return: (N, 3) ndarray in camera coordinate frame
    """
    return np.stack([points_pcd[:, 1], -points_pcd[:, 2], points_pcd[:, 0]], axis=1)

def project_pcl_to_image(pcl, t_camera_radar, camera_projection_matrix, image_shape):
    """
    Projects point cloud to camera image plane.
    :param pcl: Point cloud DataFrame with columns ['x', 'y', 'z'].
    :param t_camera_radar: Transformation matrix from radar/lidar to camera frame.
    :param camera_projection_matrix: Camera intrinsic matrix.
    :param image_shape: Shape of the camera image (height, width, channels).
    :return: uvs: 2D pixel coordinates of projected points.
             point_depth: Depth of the points in camera frame.
             power: Radar/Lidar power values.
    """
    location = np.hstack((pcl[['x', 'y', 'z']], np.ones((pcl.shape[0], 1))))  # (N, 4)
    points_camera_frame = (t_camera_radar @ location.T).T  # (N, 4)

    print(f"[Debug] Camera frame Z min/max: {points_camera_frame[:,2].min():.3f} ~ {points_camera_frame[:,2].max():.3f}")
    print(f"[Debug] Camera frame X min/max: {points_camera_frame[:,0].min():.3f} ~ {points_camera_frame[:,0].max():.3f}")
    print(f"[Debug] Camera frame Y min/max: {points_camera_frame[:,1].min():.3f} ~ {points_camera_frame[:,1].max():.3f}")

    points_xyz = points_camera_frame[:, :3].T  # (3, N)
    depth = points_xyz[2, :]
    valid = depth > 0
    points_xyz = points_xyz[:, valid]
    depth = depth[valid]

    uvs = view_points(points_xyz, camera_projection_matrix).T[:, :2]  # (N, 2)
    uvs = np.round(uvs).astype(np.int32)

    print(f"[Debug] Sample projected points (Z>0): {uvs[:10]}")

    power = pcl['rcs'].values if 'rcs' in pcl else np.ones(len(depth))
    power = power[valid]

    h, w = image_shape[:2]
    print(f"[Debug] Image size: {w}x{h}")
    in_bounds = (uvs[:, 0] >= 0) & (uvs[:, 0] < w) & (uvs[:, 1] >= 0) & (uvs[:, 1] < h)
    uvs = uvs[in_bounds]
    depth = depth[in_bounds]
    power = power[in_bounds]

    print(f"[Debug] Projected points: {len(uvs)} valid / {pcl.shape[0]} total")
    return uvs, depth, power

