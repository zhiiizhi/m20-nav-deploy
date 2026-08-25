# M20 LiDAR 仿真配置

## 概述
为云深处山猫M20机器狗添加了MuJoCo仿真中的激光雷达传感器，使用静态ray casting方法生成3D点云。

## LiDAR参数

### 安装位置
- **X**: 320.28 mm (前方)
- **Y**: 0 mm (中心)
- **Z**: -13 mm (底部)

相对于身体坐标系原点。

### 传感器规格
- **水平波束数**: 16
- **垂直波束数**: 16
- **水平视场角**: ±60° (总共120°)
- **垂直视场角**: ±15° (总共30°)
- **最大探测距离**: 30米
- **测量噪声**: 0.01米

### 总波束数
16 × 16 = **256个波束**

## 实现方法

### 1. XML模型
在 `M20.xml` 中定义了激光雷达安装位置的site：
```xml
<site name="lidar_site" pos="0.32028 0 -0.013" size="0.01" rgba="0 1 0 1"/>
```

### 2. 点云生成
使用MuJoCo的 `mj_ray` 函数进行射线检测：
- 在Python中预计算256个波束的方向向量
- 每个仿真周期将本地坐标系的波束方向转换到世界坐标系
- 使用 `mj_ray` 检测每个方向的障碍物距离
- 将有效的命中点转换为3D点云

### 3. ROS2发布
- **话题名称**: `/lidar/pointcloud`
- **消息类型**: `sensor_msgs/PointCloud2`
- **发布频率**: 10 Hz
- **坐标系**: `lidar_link`

## 使用方法

### 启动仿真
```bash
cd /home/admi/sdk_deploy/src/M20_sdk_deploy
source /opt/ros/humble/setup.bash
python3 interface/robot/simulation/mujoco_simulation_ros2.py
```

### 查看点云
```bash
# 启动rviz2
rviz2

# 添加PointCloud2显示
# 1. 点击 "Add" -> "PointCloud2"
# 2. Topic选择: /lidar/pointcloud
# 3. Fixed Frame设置为: lidar_link 或 base_link
```

### 验证点云数据
```bash
# 查看话题信息
ros2 topic info /lidar/pointcloud

# 监听点云消息
ros2 topic echo /lidar/pointcloud --once

# 查看点云统计
ros2 run topic_tools info /lidar/pointcloud
```

## 文件结构

```
M20_sdk_deploy/
├── M20_description/m20_mjcf/mjcf/
│   └── M20.xml                    # 机器人模型（包含lidar_site定义）
└── interface/robot/simulation/
    ├── mujoco_simulation_ros2.py   # 主仿真脚本（集成点云发布）
    └── lidar_point_cloud.py        # LiDAR点云生成工具类
```

## 配置调整

### 修改波束密度
在 `mujoco_simulation_ros2.py` 中修改：
```python
self.lidar = LidarPointCloud(
    horizontal_beams=32,  # 增加水平波束数
    vertical_beams=32,    # 增加垂直波束数
    ...
)
```

### 修改视场角
```python
self.lidar = LidarPointCloud(
    horizontal_fov_deg=180,  # 水平180度
    vertical_fov_deg=60,     # 垂直60度
    ...
)
```

### 修改安装位置
```python
self.lidar = LidarPointCloud(
    lidar_pos=np.array([X, Y, Z]),  # 单位：米
    ...
)
```

## 技术说明

### 为什么使用 mj_ray 而不是 rangefinder 传感器？

MuJoCo的 `<rangefinder>` 传感器沿site的本地Z轴测量距离。要实现多方向激光雷达，需要：
1. 为每个波束创建独立的site（256个site）
2. 每个site设置不同的四元数朝向
3. XML文件会变得非常冗长

使用 `mj_ray` 函数的优势：
- 更灵活：可以在Python中动态计算任意方向
- 更简洁：XML文件保持干净
- 更易维护：修改参数只需改Python代码
- 性能相当：底层使用相同的射线检测机制

### 点云坐标系
- 点云数据在 `lidar_link` 坐标系下发布
- 需要在rviz2中设置正确的TF变换或使用 `base_link` 查看

## 注意事项

1. **性能**: 256个波束每次射线检测会增加计算开销，如需更高性能可减少波束数
2. **碰撞几何**: 点云质量取决于场景中的collision geom，确保地形和障碍物有正确的碰撞体
3. **TF变换**: 在rviz2中查看点云需要正确的TF树，可能需要添加 `lidar_link` 到 `base_link` 的静态变换
