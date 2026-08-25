"""
 * @file mujoco_simulation.py
 * @brief simulation in mujoco
 * @author Bo (Percy) Peng
 * @version 1.0
 * @date 2025-11-05
 *
 * @copyright Copyright (c) 2025  DeepRobotics
"""

import os
import time
import socket
import struct
import threading
from pathlib import Path
from scipy.spatial.transform import Rotation
import numpy as np
import mujoco
import mujoco.viewer

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Time
from std_msgs.msg import Header
from geometry_msgs.msg import TransformStamped
from drdds.msg import ImuData, JointsData, JointsDataCmd, MetaType, ImuDataValue, JointsDataValue, JointData, JointDataCmd
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import TransformBroadcaster
from lidar_point_cloud import LidarPointCloud



MODEL_NAME = "M20"
# Get the directory of the current Python file
CURRENT_DIR = Path(__file__).resolve().parent

# Define the XML path relative to the Python file
XML_PATH = CURRENT_DIR / ".." / ".." / ".." / "M20_description" / "m20_mjcf" / "mjcf" / "M20_stair.xml"

# Convert to absolute path as string
XML_PATH = str(XML_PATH.resolve())
USE_VIEWER = True
DT = 0.001
RENDER_INTERVAL = 50

# Calibaration parameters (for sim-to-real consistency)
JOINT_DIR = np.array([1, 1, -1, 1, 1, -1, 1, -1, -1, 1, -1, 1, -1, -1, 1, -1], dtype=np.float32)
POS_OFFSET_DEG = np.array([-25, 229, 160, 0, 25, -131, -200, 0, -25, -229, -160, 0, 25, 131, 200, 0], dtype=np.float32)
POS_OFFSET_RAD = POS_OFFSET_DEG / 180.0 * np.pi

JOINT_INIT = {
    "M20": np.array([-0.438, -1.16, 2.76, 0,
                     0.438, -1.16, 2.76, 0,
                     -0.438, 1.16, -2.76, 0,
                     0.438, 1.16, -2.76, 0], dtype=np.float32),
}


class MuJoCoSimulationNode(Node):
    def __init__(self,
                 model_key: str = MODEL_NAME,
                 xml_path: str = XML_PATH):

        super().__init__('mujoco_simulation')

        # 加载 MJCF
        if not os.path.isfile(xml_path):
            raise FileNotFoundError(f"Cannot find MJCF: {xml_path}")

        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.model.opt.timestep = DT
        self.data = mujoco.MjData(self.model)

        # 为 LiDAR 创建独立的 mjData 副本（每个批次一个，预创建避免每帧分配）
        # front/rear 各有 num_batches 个缓冲区，用于并行计算
        self._lidar_front_num_batches = 8
        self._lidar_rear_num_batches = 8

        self.lidar_front_data = [mujoco.MjData(self.model) for _ in range(self._lidar_front_num_batches)]
        self.lidar_rear_data = [mujoco.MjData(self.model) for _ in range(self._lidar_rear_num_batches)]

        # 两个独立的 Condition 变量，分别控制前向和后向 LiDAR
        self._lidar_front_cond = threading.Condition()
        self._lidar_rear_cond = threading.Condition()
        self._lidar_front_idx = 0
        self._lidar_rear_idx = 0
        self._lidar_front_ready = True
        self._lidar_rear_ready = True
        self._lidar_running = True
        self._lidar_thread = None
        self._lidar_timestamp = 0.0
        # Shared results (accessed under lock / after workers complete)
        self._lidar_front_result = np.empty((0, 3), dtype=np.float64)
        self._lidar_rear_result = np.empty((0, 3), dtype=np.float64)
        self._lidar_front_ts = 0.0
        self._lidar_rear_ts = 0.0
        self._lidar_front_compute_done = False
        self._lidar_rear_compute_done = False

        # 机器人自由度列表
        self.actuator_ids = [a for a in range(self.model.nu)]  # 0..15
        self.dof_num = len(self.actuator_ids)
        assert self.dof_num == 16, "Expected 16 DOF for M20"

        # 初始化站立姿态
        self._set_initial_pose(model_key)

        # 缓存
        self.kp_cmd = np.zeros((self.dof_num, 1), np.float32)
        self.kd_cmd = np.zeros_like(self.kp_cmd)
        self.pos_cmd = np.zeros_like(self.kp_cmd)
        self.vel_cmd = np.zeros_like(self.kp_cmd)
        self.tau_ff = np.zeros_like(self.kp_cmd)
        self.input_tq = np.zeros_like(self.kp_cmd)

        # IMU
        self.last_base_linvel = np.zeros((3, 1), np.float64)
        self.timestamp = 0.0

        self.get_logger().info(f"[INFO] MuJoCo model loaded, dof = {self.dof_num}")

        # ROS Publishers
        self.imu_pub = self.create_publisher(ImuData, '/IMU_DATA', 200)
        self.joints_pub = self.create_publisher(JointsData, '/JOINTS_DATA', 200)
        self.pointcloud_front_pub = self.create_publisher(PointCloud2, '/lidar/front/pointcloud', 10)
        self.pointcloud_rear_pub = self.create_publisher(PointCloud2, '/lidar/rear/pointcloud', 10)
        # Lightning-LM expects /LIDAR/POINTS (RoboSense-compatible topic name)
        self.pointcloud_merged_pub = self.create_publisher(PointCloud2, '/LIDAR/POINTS', 10)

        # TF Broadcaster for LiDAR frames
        self.tf_broadcaster = TransformBroadcaster(self)

        # Initialize Front LiDAR - Front-facing 180° 96-line LiDAR
        # pitch_offset_deg=-5: 向下倾斜 5°，更好地看到地面和近距离障碍物
        self.lidar_front = LidarPointCloud(
            horizontal_beams=360,    # 0.5° horizontal resolution (180° FOV)
            vertical_beams=96,       # Full 96-line
            horizontal_fov_deg=180,  # Front only (-90° to +90°)
            vertical_fov_deg=50,     # -25° to +25°
            max_range=50.0,          # 50m max range
            lidar_pos=np.array([0.37028, 0.0, 0.013]),  # Front position
            pitch_offset_deg=0      # 向下俯仰 5°
        )

        # Initialize Rear LiDAR - Rear-facing 180° 96-line LiDAR
        self.lidar_rear = LidarPointCloud(
            horizontal_beams=360,    # 0.5° horizontal resolution (180° FOV)
            vertical_beams=96,       # Full 96-line
            horizontal_fov_deg=180,  # Rear only (-90° to +90°, relative to rear)
            vertical_fov_deg=50,     # -25° to +25°
            max_range=50.0,          # 50m max range
            lidar_pos=np.array([-0.37028, 0.0, 0.013]),  # Rear position
            yaw_offset_deg=180.0,    # Rotate 180° to face backwards
            pitch_offset_deg=0      # 向下俯仰 5°（与前向一致）
        )

        self.lidar_publish_rate = 10  # Hz, publish pointcloud at 10Hz
        self.lidar_step_counter = 0
        self.lidar_initialized = False

        # LiDAR 并行计算配置
        # 32 核 CPU: 前向 8 批 + 后向 1 批 = 9 并行任务（每个 LiDAR 内部再并行）
        self._lidar_front_num_batches = 8
        self._lidar_rear_num_batches = 8  # 

        # ROS Subscriber
        self.cmd_sub = self.create_subscription(
            JointsDataCmd,
            '/JOINTS_CMD',
            self._cmd_callback,
            50
        )

        # 可视化 - 延迟到start()中初始化
        self.viewer = None

    def _set_initial_pose(self, key: str):
        """关节位置设置为与 PyBullet 脚本一致的初始角度"""
        qpos0 = self.data.qpos.copy()
        qpos0[7:7 + self.dof_num] = JOINT_INIT[key]  # ,3-6 basequat，0-2 basepos
        qpos0[:3] = np.array([0, 10, 1.9])
        qpos0[3:7] = np.array([1, 0, 0, 0])
        self.data.qpos[:] = qpos0
        mujoco.mj_forward(self.model, self.data)

    def _cmd_callback(self, msg: JointsDataCmd):
        """Convert received (published) positions/velocities to internal (raw)"""
        if len(msg.data.joints_data) != 16:
            self.get_logger().warn("Received JointsDataCmd with incorrect number of joints")
            return

        pub_pos = np.zeros(self.dof_num, dtype=np.float32)
        pub_vel = np.zeros(self.dof_num, dtype=np.float32)
        for i in range(self.dof_num):
            joint_cmd = msg.data.joints_data[i]
            self.kp_cmd[i] = joint_cmd.kp
            self.kd_cmd[i] = joint_cmd.kd
            pub_pos[i] = joint_cmd.position
            pub_vel[i] = joint_cmd.velocity
            self.tau_ff[i] = joint_cmd.torque  # tau_ff no processing

        # Convert: raw = published * dir + offset_rad
        self.pos_cmd.flat = pub_pos * JOINT_DIR + POS_OFFSET_RAD
        self.vel_cmd.flat = pub_vel * JOINT_DIR

    def start(self):
        # LiDAR 预热（使用单线程测试）
        self.get_logger().info("Warming up LiDAR ray casting...")
        try:
            # 初始化测试用的 MjData
            test_data = mujoco.MjData(self.model)
            test_data.qpos[:] = self.data.qpos[:]
            test_data.qvel[:] = self.data.qvel[:]
            test_data.ctrl[:] = self.data.ctrl[:]
            mujoco.mj_kinematics(self.model, test_data)

            test_points_front = self.lidar_front.cast_rays(self.model, test_data)
            valid_count_front = test_points_front.shape[0]

            test_points_rear = self.lidar_rear.cast_rays(self.model, test_data)
            valid_count_rear = test_points_rear.shape[0]

            self.lidar_initialized = True
            self.get_logger().info(f"Front LiDAR warmed up: {valid_count_front}/{self.lidar_front.horizontal_beams * self.lidar_front.vertical_beams} beams valid")
            self.get_logger().info(f"Rear LiDAR warmed up: {valid_count_rear}/{self.lidar_rear.horizontal_beams * self.lidar_rear.vertical_beams} beams valid")
        except Exception as e:
            self.get_logger().warn(f"LiDAR warm-up failed: {e}")
            self.lidar_initialized = False

        # 初始化viewer（延迟到这里以避免OpenGL上下文问题）
        if USE_VIEWER:
            self.get_logger().info("Initializing MuJoCo viewer...")
            self.viewer = mujoco.viewer.launch_passive(self.model, self.data)

        # viewer 初始化完成后，启动 LiDAR 工作线程（两个并行线程：前向 + 后向）
        self._lidar_front_thread = threading.Thread(
            target=self._lidar_worker_single, args=("front",), daemon=True, name="LiDAR-Front-Worker"
        )
        self._lidar_rear_thread = threading.Thread(
            target=self._lidar_worker_single, args=("rear",), daemon=True, name="LiDAR-Rear-Worker"
        )
        self._lidar_front_thread.start()
        self._lidar_rear_thread.start()
        self.get_logger().info("LiDAR worker threads started (front + rear parallel, merged by main thread)")

        # 提前发布一次 LiDAR TF，供 RKO‑LIO 启动时查询
        self._publish_lidar_tf()
        # 发布静态 odom→base_link TF 兜底（RKO‑LIO odometry 就绪后由 relay 覆盖）
        self._publish_odom_base_tf()
        self.get_logger().info("Static LiDAR TF published at startup")

        # 主模拟循环
        step = 0
        last_time = time.perf_counter()
        loop_count = 0
        loop_start = time.perf_counter()
        last_lidar_print = step
        mj_step_total = 0.0
        viewer_sync_total = 0.0
        spin_total = 0.0
        skipped = 0
        while rclpy.ok():
            now = time.perf_counter()

            if now - last_time >= DT:
                last_time = now
                step += 1

                t_step_start = time.perf_counter()
                # 控制律
                self._apply_joint_torque()
                # 模拟一步（不再需要锁，lidar_data 是独立的）
                mujoco.mj_step(self.model, self.data)
                mj_step_total += time.perf_counter() - t_step_start

                self.timestamp = step * DT

                # 采样 & 发送观测 (every 5 steps for 200 Hz)
                if step % 5 == 0:
                    self._publish_robot_state(step)

                # 可视化 — 计时
                if self.viewer and step % RENDER_INTERVAL == 0:
                    t_viewer_start = time.perf_counter()
                    self.viewer.sync()
                    viewer_sync_total += time.perf_counter() - t_viewer_start

                # 每1000次主循环打印一次平均速度
                loop_count += 1
                if loop_count % 1000 == 0:
                    loop_elapsed = time.perf_counter() - loop_start
                    loop_start = time.perf_counter()
                    avg_step_ms = mj_step_total / 1000 * 1000
                    avg_spin_ms = spin_total / 1000 * 1000
                    n_viewer_syncs = 1000 // RENDER_INTERVAL
                    avg_viewer_ms = viewer_sync_total / n_viewer_syncs * 1000 if n_viewer_syncs > 0 else 0
                    mj_step_total = 0.0
                    viewer_sync_total = 0.0
                    spin_total = 0.0
                    steps_done = step - last_lidar_print
                    last_lidar_print = step
                    sim_step_hz = 1000.0 / loop_elapsed if loop_elapsed > 0 else 0
                    lidar_actual_hz = steps_done / loop_elapsed if loop_elapsed > 0 else 0
                    print(f"[PERF] 1000iter/{loop_elapsed:.3f}s | step={avg_step_ms:.3f}ms | viewer={avg_viewer_ms:.1f}ms | spin={avg_spin_ms:.3f}ms | skipped={skipped} | sim_hz={sim_step_hz:.0f} | lidar_hz={lidar_actual_hz:.1f}")
                    loop_count = 0
                    skipped = 0
            else:
                skipped += 1

            # Handle ROS callbacks — non-blocking
            t_spin_start = time.perf_counter()
            rclpy.spin_once(self, timeout_sec=0.0)
            spin_total += time.perf_counter() - t_spin_start

    def _apply_joint_torque(self):
        # 当前关节状态
        q = self.data.qpos[7:7 + self.dof_num].reshape(-1, 1)
        dq = self.data.qvel[6:6 + self.dof_num].reshape(-1, 1)
        self.input_tq = (
                self.kp_cmd * (self.pos_cmd - q) +
                self.kd_cmd * (self.vel_cmd - dq) +
                self.tau_ff
        )

        # 写入 control 缓冲区
        self.data.ctrl[:] = self.input_tq.flatten()

    # --------------------------------------------------------
    def quaternion_to_euler(self, q):
        """
        Convert a quaternion to Euler angles (roll, pitch, yaw).
        """
        w, x, y, z = q

        # roll (X-axis rotation)
        t0 = 2.0 * (w * x + y * z)
        t1 = 1.0 - 2.0 * (x * x + y * y)
        roll = np.arctan2(t0, t1)

        # pitch (Y-axis rotation)
        t2 = 2.0 * (w * y - z * x)
        t2 = np.clip(t2, -1.0, 1.0)  # 防止数值漂移导致 |t2|>1
        pitch = np.arcsin(t2)

        # yaw (Z-axis rotation)
        t3 = 2.0 * (w * z + x * y)
        t4 = 1.0 - 2.0 * (y * y + z * z)
        yaw = np.arctan2(t3, t4)

        return np.array([roll, pitch, yaw], dtype=np.float32)

    # --------------------------------------------------------

    def _publish_robot_state(self, step: int):
        # ----- IMU -----
        q_world = self.data.sensordata[:4]  # quaternion (w, x, y, z) in MuJoCo convention
        rpy_rad = self.quaternion_to_euler(q_world)  # returns [roll, pitch, yaw] in radians

        # Convert to degrees
        rpy_deg = [angle * (180.0 / 3.141592653589793) for angle in rpy_rad]

        body_acc = self.data.sensordata[4:7]
        angvel_b = self.data.sensordata[7:10]  # body frame

        imu_msg = ImuData()
        imu_msg.header = MetaType()
        imu_msg.header.frame_id = 0
        stamp = Time()
        sec = int(self.timestamp)
        nanosec = int((self.timestamp - sec) * 1e9)
        stamp.sec = sec
        stamp.nanosec = nanosec
        imu_msg.header.stamp = stamp
        imu_msg.data = ImuDataValue()
        imu_msg.data.roll = float(rpy_deg[0])
        imu_msg.data.pitch = float(rpy_deg[1])
        imu_msg.data.yaw = float(rpy_deg[2])
        imu_msg.data.omega_x = float(angvel_b[0])
        imu_msg.data.omega_y = float(angvel_b[1])
        imu_msg.data.omega_z = float(angvel_b[2])
        imu_msg.data.acc_x = float(body_acc[0])
        imu_msg.data.acc_y = float(body_acc[1])
        imu_msg.data.acc_z = float(body_acc[2])
        self.imu_pub.publish(imu_msg)

        # ----- 关节 -----
        q = self.data.qpos[7:7 + self.dof_num]
        dq = self.data.qvel[6:6 + self.dof_num]
        tau = self.input_tq.flatten()

        # Convert raw to published: published = (raw - offset_rad) * dir
        pub_pos = (q - POS_OFFSET_RAD) * JOINT_DIR
        pub_vel = dq * JOINT_DIR
        pub_tau = tau * JOINT_DIR  # Torque also needs direction flip
        
        joints_msg = JointsData()
        joints_msg.header = MetaType()
        joints_msg.header.frame_id = 0
        stamp = Time()
        sec = int(self.timestamp)
        nanosec = int((self.timestamp - sec) * 1e9)
        stamp.sec = sec
        stamp.nanosec = nanosec
        joints_msg.header.stamp = stamp
        joints_msg.data = JointsDataValue()
        joints_msg.data.joints_data = [JointData() for _ in range(self.dof_num)]
        for i in range(self.dof_num):
            joint = joints_msg.data.joints_data[i]
            joint.name = [32, 32, 32, 32]  # Dummy name (four spaces)
            joint.data_id = 0  # Dummy
            joint.status_word = 1  # Normal
            joint.position = float(pub_pos[i])
            joint.torque = float(pub_tau[i])
            joint.velocity = float(pub_vel[i])
            joint.motion_temp = 40.0  # Dummy normal temp
            joint.driver_temp = 45.0  # Dummy normal temp
        self.joints_pub.publish(joints_msg)

        # ----- LiDAR PointCloud -----
        # _publish_robot_state 每 5 步调用一次, 所以 lidar_interval 要除以 5
        self.lidar_step_counter += 1
        lidar_interval = int(1.0 / (DT * 5 * self.lidar_publish_rate))
        if self.lidar_step_counter % lidar_interval == 0:
            t_sync_start = time.perf_counter()
            # 同步时间戳到所有 LiDAR 批次缓冲区
            self._lidar_timestamp = self.timestamp

            # 前向 LiDAR：等待 worker 完成上一帧
            t_wait_front_start = time.perf_counter()
            with self._lidar_front_cond:
                got_front = self._lidar_front_cond.wait_for(
                    lambda: self._lidar_front_ready, timeout=2.0
                )
            t_wait_front_end = time.perf_counter()
            wait_front_time = t_wait_front_end - t_wait_front_start
            if wait_front_time > 0.001:
                print(f"[WAIT] Front LiDAR waited {wait_front_time:.4f}s for previous frame")

            # 前向 LiDAR：同步所有 8 个批次缓冲区
            t_kin_front_start = time.perf_counter()
            t_kin_single = []
            with self._lidar_front_cond:
                if self._lidar_front_ready:
                    for i in range(self._lidar_front_num_batches):
                        t_k_single_start = time.perf_counter()
                        dst = self.lidar_front_data[i]
                        dst.qpos[:] = self.data.qpos[:]
                        dst.qvel[:] = self.data.qvel[:]
                        dst.ctrl[:] = self.data.ctrl[:]
                        mujoco.mj_kinematics(self.model, dst)
                        t_k_single_end = time.perf_counter()
                        t_kin_single.append(t_k_single_end - t_k_single_start)
                    t_kin_front_end = time.perf_counter()
                    self._lidar_front_ready = False
                    self._lidar_front_cond.notify()
                    print(f"[PERF] Front LiDAR sync: mj_kinematics x{self._lidar_front_num_batches} = {t_kin_front_end-t_kin_front_start:.4f}s (single: min={min(t_kin_single):.4f}s, max={max(t_kin_single):.4f}s)")

            # 后向 LiDAR：等待 worker 完成上一帧
            t_wait_rear_start = time.perf_counter()
            with self._lidar_rear_cond:
                got_rear = self._lidar_rear_cond.wait_for(
                    lambda: self._lidar_rear_ready, timeout=2.0
                )
            t_wait_rear_end = time.perf_counter()
            wait_rear_time = t_wait_rear_end - t_wait_rear_start
            if wait_rear_time > 0.001:
                print(f"[WAIT] Rear LiDAR waited {wait_rear_time:.4f}s for previous frame")

            # 后向 LiDAR：同步所有 8 个批次缓冲区
            t_kin_rear_start = time.perf_counter()
            t_kin_single_rear = []
            with self._lidar_rear_cond:
                if self._lidar_rear_ready:
                    for i in range(self._lidar_rear_num_batches):
                        t_k_single_start = time.perf_counter()
                        dst = self.lidar_rear_data[i]
                        dst.qpos[:] = self.data.qpos[:]
                        dst.qvel[:] = self.data.qvel[:]
                        dst.ctrl[:] = self.data.ctrl[:]
                        mujoco.mj_kinematics(self.model, dst)
                        t_k_single_end = time.perf_counter()
                        t_kin_single_rear.append(t_k_single_end - t_k_single_start)
                    t_kin_rear_end = time.perf_counter()
                    self._lidar_rear_ready = False
                    self._lidar_rear_cond.notify()
                    print(f"[PERF] Rear LiDAR sync: mj_kinematics x{self._lidar_rear_num_batches} = {t_kin_rear_end-t_kin_rear_start:.4f}s (single: min={min(t_kin_single_rear):.4f}s, max={max(t_kin_single_rear):.4f}s)")

            t_sync_end = time.perf_counter()
            print(f"[PERF] Total LiDAR sync (main thread): {t_sync_end-t_sync_start:.4f}s (wait_front={wait_front_time:.4f}s, wait_rear={wait_rear_time:.4f}s)")

            # 主线程统一发布 merged 点云（保证时间同步）
            self._publish_merged_from_workers()

    def _publish_merged_from_workers(self):
        """主线程统一发布 merged 点云，确保前后点云使用同一时间戳"""
        # 获取两个 worker 的结果
        points_front = self._lidar_front_result
        points_rear = self._lidar_rear_result

        # 坐标变换（相对于 base_link 中心）
        front_offset = np.array([0.37028, 0.0, 0.013])
        rear_offset = np.array([-0.37028, 0.0, 0.013])

        points_front_merged = points_front + front_offset if len(points_front) > 0 else np.empty((0, 3))
        points_rear_merged = points_rear + rear_offset if len(points_rear) > 0 else np.empty((0, 3))

        if len(points_front_merged) > 0 or len(points_rear_merged) > 0:
            merged_points = np.concatenate([points_front_merged, points_rear_merged], axis=0)

            # 统一时间戳
            merge_ts = max(self._lidar_front_ts, self._lidar_rear_ts)

            header = Header()
            stamp_sec = int(merge_ts)
            stamp_nanosec = int((merge_ts - stamp_sec) * 1e9)
            header.stamp.sec = stamp_sec
            header.stamp.nanosec = stamp_nanosec
            header.frame_id = "lidar_link"

            cloud_merged = point_cloud2.create_cloud_xyz32(header, merged_points)
            self.pointcloud_merged_pub.publish(cloud_merged)

            # 跟踪发布频率
            if not hasattr(self, '_last_merged_pub_time'):
                self._last_merged_pub_time = None
                self._merged_pub_count = 0
            now = time.perf_counter()
            self._merged_pub_count += 1
            if self._last_merged_pub_time is not None:
                delta = now - self._last_merged_pub_time
                if self._merged_pub_count % 10 == 0:
                    print(f"[MERGED] pub delta={delta:.4f}s (~{1.0/delta:.1f}Hz), points={len(merged_points)}")
            self._last_merged_pub_time = now

        # 发布 TF
        stamp = self._make_time_msg()
        self._publish_lidar_tf(stamp)

    def _make_time_msg(self):
        t = Time()
        sec = int(self.timestamp)
        t.sec = sec
        t.nanosec = int((self.timestamp - sec) * 1e9)
        return t

    def _publish_lidar_tf(self, stamp=None):
        """Publish TF transforms for LiDAR frames"""
        from geometry_msgs.msg import TransformStamped
        if stamp is None:
            stamp = self._make_time_msg()

        # Front LiDAR: base_link -> front_lidar_link
        tf_front = TransformStamped()
        tf_front.header.stamp = stamp
        tf_front.header.frame_id = "base_link"
        tf_front.child_frame_id = "front_lidar_link"
        tf_front.transform.translation.x = 0.37028
        tf_front.transform.translation.y = 0.0
        tf_front.transform.translation.z = 0.013
        tf_front.transform.rotation.x = 0.0
        tf_front.transform.rotation.y = 0.0
        tf_front.transform.rotation.z = 0.0
        tf_front.transform.rotation.w = 1.0

        # Rear LiDAR: base_link -> rear_lidar_link
        tf_rear = TransformStamped()
        tf_rear.header.stamp = stamp
        tf_rear.header.frame_id = "base_link"
        tf_rear.child_frame_id = "rear_lidar_link"
        tf_rear.transform.translation.x = -0.37028
        tf_rear.transform.translation.y = 0.0
        tf_rear.transform.translation.z = 0.013
        tf_rear.transform.rotation.x = 0.0
        tf_rear.transform.rotation.y = 0.0
        tf_rear.transform.rotation.z = 0.0
        tf_rear.transform.rotation.w = 1.0

        # Merged LiDAR: base_link -> lidar_link (at center)
        tf_merged = TransformStamped()
        tf_merged.header.stamp = stamp
        tf_merged.header.frame_id = "base_link"
        tf_merged.child_frame_id = "lidar_link"
        tf_merged.transform.translation.x = 0.0
        tf_merged.transform.translation.y = 0.0
        tf_merged.transform.translation.z = 0.0
        tf_merged.transform.rotation.x = 0.0
        tf_merged.transform.rotation.y = 0.0
        tf_merged.transform.rotation.z = 0.0
        tf_merged.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(tf_front)
        self.tf_broadcaster.sendTransform(tf_rear)
        self.tf_broadcaster.sendTransform(tf_merged)

    def _publish_odom_base_tf(self, stamp=None):
        """Publish static odom→base_link TF (identity) so RViz can resolve transforms
        before RKO-LIO odometry is available."""
        if stamp is None:
            stamp = self._make_time_msg()
        tf_odom = TransformStamped()
        tf_odom.header.stamp = stamp
        tf_odom.header.frame_id = "odom"
        tf_odom.child_frame_id = "base_link"
        tf_odom.transform.translation.x = 0.0
        tf_odom.transform.translation.y = 0.0
        tf_odom.transform.translation.z = 0.0
        tf_odom.transform.rotation.x = 0.0
        tf_odom.transform.rotation.y = 0.0
        tf_odom.transform.rotation.z = 0.0
        tf_odom.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(tf_odom)

    def _lidar_worker_single(self, direction):
        """
        单个 LiDAR（前向或后向）的工作线程。
        同步模式：
        1. 等待 ready=False（主线程已同步所有批次缓冲区）
        2. 在锁外执行 cast_rays_parallel（使用所有批次并行计算）
        3. 完成后获取锁，标记 ready=True，通知主线程
        """
        if direction == "front":
            lidar = self.lidar_front
            cond = self._lidar_front_cond
            data_list = self.lidar_front_data
            num_batches = self._lidar_front_num_batches
        else:
            lidar = self.lidar_rear
            cond = self._lidar_rear_cond
            data_list = self.lidar_rear_data
            num_batches = self._lidar_rear_num_batches

        self.get_logger().info(f"LiDAR {direction} worker thread running (num_batches={num_batches})")

        while self._lidar_running:
            ts = 0.0

            # 等待主线程同步所有批次缓冲区
            with cond:
                got_signal = False
                if direction == "front":
                    got_signal = cond.wait_for(
                        lambda: not self._lidar_front_ready or not self._lidar_running,
                        timeout=1.0
                    )
                    if got_signal and self._lidar_running:
                        ts = self._lidar_timestamp
                else:
                    got_signal = cond.wait_for(
                        lambda: not self._lidar_rear_ready or not self._lidar_running,
                        timeout=1.0
                    )
                    if got_signal and self._lidar_running:
                        ts = self._lidar_timestamp

                if not got_signal or not self._lidar_running:
                    continue

            # 在锁外执行 cast_rays_parallel（使用所有预创建的 data_copies）
            t_compute_start = time.perf_counter()
            try:
                points = lidar.cast_rays_parallel(self.model, data_list, num_batches=num_batches)
            except Exception as e:
                self.get_logger().warn(f"LiDAR {direction} cast_rays error: {e}")
                with cond:
                    if direction == "front":
                        self._lidar_front_ready = True
                    else:
                        self._lidar_rear_ready = True
                    cond.notify()
                continue
            t_compute_end = time.perf_counter()
            compute_time = t_compute_end - t_compute_start
            print(f"[WORKER {direction.upper()}] cast_rays_parallel={compute_time:.4f}s")

            # 存储结果到共享变量（不发布，等主线程统一发布）
            if direction == "front":
                self._lidar_front_result = points
                self._lidar_front_ts = ts
                self._lidar_front_compute_done = True
            else:
                self._lidar_rear_result = points
                self._lidar_rear_ts = ts
                self._lidar_rear_compute_done = True

            # 完成，通知主线程可以同步下一帧
            with cond:
                if direction == "front":
                    self._lidar_front_ready = True
                else:
                    self._lidar_rear_ready = True
                cond.notify()

        self.get_logger().info(f"LiDAR {direction} worker thread stopped")

    def _publish_single_pointcloud(self, points, ts, direction):
        """不再在此发布（改为在 _publish_merged_from_workers 中统一发布）"""
        pass

    def shutdown(self):
        """清理资源，停止 LiDAR 线程"""
        self.get_logger().info("Shutting down LiDAR worker threads...")
        self._lidar_running = False
        with self._lidar_front_cond:
            self._lidar_front_cond.notify_all()
        with self._lidar_rear_cond:
            self._lidar_rear_cond.notify_all()
        if hasattr(self, '_lidar_front_thread'):
            self._lidar_front_thread.join(timeout=3.0)
        if hasattr(self, '_lidar_rear_thread'):
            self._lidar_rear_thread.join(timeout=3.0)
        if self.viewer:
            self.viewer.close()
        self.get_logger().info("LiDAR worker threads stopped")


if __name__ == "__main__":
    np.set_printoptions(precision=4, suppress=True)
    rclpy.init()
    sim_node = MuJoCoSimulationNode()
    try:
        sim_node.start()
    except KeyboardInterrupt:
        pass
    finally:
        sim_node.shutdown()
        sim_node.destroy_node()
        rclpy.shutdown()