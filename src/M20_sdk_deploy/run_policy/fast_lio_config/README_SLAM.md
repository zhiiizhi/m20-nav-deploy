# M20 机器狗 3D LiDAR SLAM 建图指南

基于 RKO-LIO（ROS2 Humble 原生 LiDAR-惯性紧耦合里程计）。

## 系统架构

```
终端1 (MuJoCo仿真)                       终端2 (IMU桥接)                 终端3 (SLAM)
┌──────────────────┐     /IMU_DATA      ┌──────────────────┐  /imu     ┌──────────────────┐
│ mujoco_simulation│ ──────────────────→│ m20_imu_bridge   │ ────────→│   RKO-LIO        │
│   (Python)       │                    │ drdds→sensor_msgs│          │   online_node    │
│                  │  /lidar/merged/     └──────────────────┘          │                  │
│  LidarPointCloud │    pointcloud                                    │  /rko_lio/       │
│   (mj_multiRay)  │ ────────────────────────────────────────────────→│    odometry      │
└──────────────────┘                                                  │  /rko_lio/       │
                                                                      │    local_map     │
                                                                      └──────────────────┘
```

## 使用方法

### 方式一：一键启动（推荐）

```bash
cd ~/sdk_deploy
source install/setup.bash

# 启动全部节点
bash src/M20_sdk_deploy/scripts/run_m20_slam.sh
```

### 方式二：多终端分别启动

**终端1 - 仿真：**
```bash
cd ~/sdk_deploy
source install/setup.bash
python3 src/M20_sdk_deploy/interface/robot/simulation/mujoco_simulation_ros2.py
```

**终端2 - IMU桥接：**
```bash
cd ~/sdk_deploy
source install/setup.bash
python3 src/M20_sdk_deploy/interface/robot/simulation/m20_imu_bridge.py
```

**终端3 - SLAM：**
```bash
source /opt/ros/humble/setup.bash
cd ~/sdk_deploy
source install/setup.bash

ros2 launch rko_lio odometry.launch.py \
    config_file:=src/M20_sdk_deploy/run_policy/fast_lio_config/m20_marsim.yaml \
    mode:=online rviz:=true
```

## 地图保存

RKO-LIO 提供了地图保存功能。需要在地图建好之后调用以下命令：

### 方法一：通过服务保存完整点云地图

```bash
# 启动时带 dump_results:=true
ros2 launch rko_lio odometry.launch.py \
    config_file:=src/M20_sdk_deploy/run_policy/fast_lio_config/m20_marsim.yaml \
    mode:=online dump_results:=true run_name:=m20_map_01

# 地图保存在 ~/results/m20_map_01/ 目录下
# 包含:
#   - trajectory.txt        # 轨迹文件
#   - config.yaml           # 配置参数
#   - map.pcd / map.bag     # 点云地图（取决于设置）
```

### 方法二：实时录制点云

在 SLAM 运行的同时，另一个终端用 ros2 bag 录制：

```bash
# 录制所有建图相关主题
ros2 bag record -o m20_slam_map /rko_lio/odometry /rko_lio/local_map /lidar/merged/pointcloud /tf /tf_static
```

### 方法三：查看地图点云

SLAM 运行中即可在 RViz 中看到增量地图：
- 固定框（Fixed Frame）设为 `odom`
- 添加 PointCloud2 显示，主题选 `/rko_lio/local_map`
- 点的大小设为 2-3 以便观察

## 地图文件格式

保存的地图为 PCD 格式（Point Cloud Data），可用以下工具查看：

```bash
# 安装 pcl-tools
sudo apt install pcl-tools

# 查看 PCD 文件
pcl_viewer map.pcd

# 或使用 Python
python3 -c "
import open3d as o3d
pcd = o3d.io.read_point_cloud('map.pcd')
o3d.visualization.draw_geometries([pcd])
"
```

## 注意事项

1. **仿真启动时机器人保持静止**：RKO-LIO 初始化阶段假设机器人静止，启动后先等几秒再移动
2. **点云坐标系**：修改后的 LiDAR 点云输出在 `lidar_link` 局部坐标系，与 `base_link` 重合（对于 merged 点云）
3. **首次建图**：建议先让机器人缓慢行走并覆盖全部区域，完成后保存地图
4. **如需 FAST-LIO2 源码编译**：我已准备好在 `src/FAST-LIO2/` 中，但需要 ROS1 环境。RKO-LIO 是 ROS2 原生等价替代方案
