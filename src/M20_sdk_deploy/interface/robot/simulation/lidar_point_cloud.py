"""
 * @file lidar_point_cloud.py
 * @brief LiDAR rangefinder simulation using MuJoCo batched ray casting
 * @author Assistant
 * @date 2026-05-19
 *
 * Uses MuJoCo mj_multiRay for batched collision ray casting (no OpenGL required)
"""

import time
import numpy as np
import mujoco
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from concurrent.futures import ThreadPoolExecutor, as_completed


class LidarPointCloud:
    """
    Simulates a 3D LiDAR using MuJoCo's mj_multiRay batched collision detection.
    Precomputes beam directions and casts all rays in a single C call.

    Simulated specs (front-facing 180° 96-line LiDAR):
    - Horizontal FOV: 180° (front only)
    - Vertical FOV: 90° (-45° to +45°)
    - Horizontal beams: 360 (0.5° resolution)
    - Vertical beams: 96 (full 96-line)
    - Total beams: 34,560
    - Publish rate: 10 Hz
    - Point frequency: ~345,600 points/sec
    """

    def __init__(self,
                 horizontal_beams=360,
                 vertical_beams=96,
                 horizontal_fov_deg=180,
                 vertical_fov_deg=90,
                 max_range=50.0,
                 lidar_pos=np.array([0.37028, 0.0, 0.013]),
                 yaw_offset_deg=0.0,
                 pitch_offset_deg=0.0):
        self.horizontal_beams = horizontal_beams
        self.vertical_beams = vertical_beams
        self.horizontal_fov = np.deg2rad(horizontal_fov_deg)
        self.vertical_fov = np.deg2rad(vertical_fov_deg)
        self.max_range = max_range
        self.lidar_pos = lidar_pos
        self.yaw_offset = np.deg2rad(yaw_offset_deg)
        self.pitch_offset = np.deg2rad(pitch_offset_deg)

        self._base_body_id = None

        self.beam_directions = self._compute_beam_directions()

    def _compute_beam_directions(self):
        if self.horizontal_fov >= 2 * np.pi - 0.01:
            h_angles = np.linspace(0, 2 * np.pi, self.horizontal_beams, endpoint=False)
        else:
            h_angles = np.linspace(-self.horizontal_fov/2, self.horizontal_fov/2, self.horizontal_beams)

        h_angles = h_angles + self.yaw_offset
        v_angles = np.linspace(-self.vertical_fov/2, self.vertical_fov/2, self.vertical_beams)

        h_grid, v_grid = np.meshgrid(h_angles, v_angles)
        cos_v = np.cos(v_grid)
        x = cos_v * np.cos(h_grid)
        y = cos_v * np.sin(h_grid)
        z = np.sin(v_grid)

        directions = np.stack([x, y, z], axis=-1).reshape(-1, 3)
        norms = np.linalg.norm(directions, axis=-1, keepdims=True)
        directions = directions / norms

        # Apply pitch offset (rotation around Y axis)
        if self.pitch_offset != 0.0:
            cp = np.cos(self.pitch_offset)
            sp = np.sin(self.pitch_offset)
            # Rotation matrix around Y axis: [cp 0 sp; 0 1 0; -sp 0 cp]
            x_new = cp * directions[:, 0] + sp * directions[:, 2]
            z_new = -sp * directions[:, 0] + cp * directions[:, 2]
            directions[:, 0] = x_new
            directions[:, 2] = z_new

        return directions

    def cast_rays(self, model, data):
        """
        Cast all rays in a single mj_multiRay call.

        Returns points in the LiDAR local frame (forward=+X, left=+Y, up=+Z),
        so the frame_id "lidar_link" / "front_lidar_link" is consistent.

        Returns:
            ndarray of shape (M, 3) with valid hit points, or empty array.
        """
        t_start = time.perf_counter()

        if self._base_body_id is None:
            self._base_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'base_link')

        base_pos = data.xpos[self._base_body_id]
        base_mat = data.xmat[self._base_body_id].reshape(3, 3)

        lidar_world_pos = base_pos + base_mat @ self.lidar_pos

        directions_world = (base_mat @ self.beam_directions.T).T
        norms = np.linalg.norm(directions_world, axis=1, keepdims=True)
        directions_world = directions_world / norms

        num_beams = len(self.beam_directions)
        dist = np.empty(num_beams, dtype=np.float64)
        geomid = np.empty(num_beams, dtype=np.int32)

        t_prep = time.perf_counter()

        mujoco.mj_multiRay(
            model, data,
            lidar_world_pos,
            directions_world.ravel(),
            None,
            1,
            self._base_body_id,
            geomid, dist,
            None,
            num_beams,
            self.max_range
        )

        t_ray = time.perf_counter()

        mask = dist > 0.01
        num_valid = np.count_nonzero(mask)
        if num_valid == 0:
            return np.empty((0, 3), dtype=np.float64)

        hit_dists = dist[mask]
        result = hit_dists[:, np.newaxis] * self.beam_directions[mask]

        t_end = time.perf_counter()
        print(f"[PERF] cast_rays: prep={t_prep-t_start:.4f}s, mj_multiRay={t_ray-t_prep:.4f}s, post={t_end-t_ray:.4f}s, total={t_end-t_start:.4f}s, beams={num_beams}, valid={num_valid}")

        return result

    def _cast_rays_batch(self, model, data, beam_indices, batch_id=None):
        """
        Cast a subset of rays (specified by beam_indices).
        Returns points in LiDAR local frame.
        """
        t_batch_start = time.perf_counter()

        if self._base_body_id is None:
            self._base_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'base_link')

        base_pos = data.xpos[self._base_body_id]
        base_mat = data.xmat[self._base_body_id].reshape(3, 3)
        lidar_world_pos = base_pos + base_mat @ self.lidar_pos

        t_prep = time.perf_counter()

        batch_directions = self.beam_directions[beam_indices]
        directions_world = (base_mat @ batch_directions.T).T
        norms = np.linalg.norm(directions_world, axis=1, keepdims=True)
        directions_world = directions_world / norms

        num_beams = len(beam_indices)
        dist = np.empty(num_beams, dtype=np.float64)
        geomid = np.empty(num_beams, dtype=np.int32)

        t_ray_start = time.perf_counter()

        mujoco.mj_multiRay(
            model, data,
            lidar_world_pos,
            directions_world.ravel(),
            None,
            1,
            self._base_body_id,
            geomid, dist,
            None,
            num_beams,
            self.max_range
        )

        t_ray_end = time.perf_counter()

        mask = dist > 0.01
        num_valid = np.count_nonzero(mask)
        if num_valid == 0:
            t_end = time.perf_counter()
            if batch_id is not None:
                print(f"[BATCH#{batch_id}] prep={t_prep-t_batch_start:.4f}s, ray={t_ray_end-t_ray_start:.4f}s, total={t_end-t_batch_start:.4f}s, beams={num_beams}, valid=0")
            return np.empty((0, 3), dtype=np.float64)

        hit_dists = dist[mask]
        result = hit_dists[:, np.newaxis] * batch_directions[mask]

        t_end = time.perf_counter()
        if batch_id is not None:
            print(f"[BATCH#{batch_id}] prep={t_prep-t_batch_start:.4f}s, ray={t_ray_end-t_ray_start:.4f}s, post={t_end-t_ray_end:.4f}s, total={t_end-t_batch_start:.4f}s, beams={num_beams}, valid={num_valid}")
        return result

    def cast_rays_parallel(self, model, data_copies, num_batches=8):
        """
        Cast rays by splitting into multiple batches and computing in parallel.
        Each batch uses a pre-created mjData copy to avoid thread conflicts.

        Args:
            model: MuJoCo model (read-only, thread-safe)
            data_copies: List of pre-created MjData copies (already initialized with kinematics)
            num_batches: Number of batches to split rays into (should match len(data_copies))

        Returns:
            ndarray of shape (M, 3) with valid hit points, or empty array.
        """
        t_start = time.perf_counter()

        total_beams = len(self.beam_directions)
        if total_beams == 0:
            return np.empty((0, 3), dtype=np.float64)

        # Ensure we have enough data copies
        actual_batches = min(num_batches, len(data_copies))

        # If too few beams, don't bother with parallel
        if total_beams < actual_batches * 10:
            return self.cast_rays(model, data_copies[0])

        # Split beams into batches
        t_split = time.perf_counter()
        batch_indices = np.array_split(np.arange(total_beams), actual_batches)

        t_submit = time.perf_counter()

        # Compute in parallel - data_copies are pre-initialized, no per-frame overhead
        all_points = []
        batch_times = {}  # Track individual batch times
        with ThreadPoolExecutor(max_workers=actual_batches) as executor:
            futures = {
                executor.submit(self._cast_rays_batch, model, data_copies[i], indices, batch_id=i): i
                for i, indices in enumerate(batch_indices)
            }
            for future in as_completed(futures):
                batch_id = futures[future]
                try:
                    points = future.result()
                    if len(points) > 0:
                        all_points.append(points)
                except Exception as e:
                    pass  # Skip failed batches

        t_done = time.perf_counter()

        if not all_points:
            return np.empty((0, 3), dtype=np.float64)

        result = np.concatenate(all_points, axis=0)
        t_end = time.perf_counter()

        print(f"[PERF] cast_rays_parallel: split={t_split-t_start:.4f}s, submit+wait={t_done-t_submit:.4f}s, concat={t_end-t_done:.4f}s, total={t_end-t_start:.4f}s, beams={total_beams}, batches={actual_batches}, valid={result.shape[0]}")
        print(f"[PARALLEL] submit→done wall time: {t_done-t_submit:.4f}s (if batches ran serial, expect ~8x single batch time)")

        return result

    def create_pointcloud_msg(self, points, timestamp, frame_id="front_lidar_link"):
        """
        Create ROS2 PointCloud2 message from ray casting results.

        Args:
            points: ndarray of shape (M, 3) or list of [x,y,z] / None
            timestamp: Current simulation timestamp in seconds
            frame_id: Coordinate frame ID for the point cloud

        Returns:
            sensor_msgs/PointCloud2 message, or None if no valid points
        """
        if isinstance(points, np.ndarray):
            if points.shape[0] == 0:
                return None
            valid_points = points
        else:
            valid_points_list = [p for p in points if p is not None]
            if not valid_points_list:
                return None
            valid_points = valid_points_list

        header = Header()
        stamp_sec = int(timestamp)
        stamp_nanosec = int((timestamp - stamp_sec) * 1e9)
        header.stamp.sec = stamp_sec
        header.stamp.nanosec = stamp_nanosec
        header.frame_id = frame_id

        cloud_msg = point_cloud2.create_cloud_xyz32(header, valid_points)
        return cloud_msg
