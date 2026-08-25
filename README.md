# M20 机器狗导航系统操作指南

## 系统架构总览

```
┌─────────────────────────────────────────────────────────────┐
│                    机器狗三主机架构                            │
├─────────────┬─────────────────┬─────────────────────────────┤
│  103 运动主机 │   106 导航主机   │      104 开发主机            │
│  basic_server│  原厂导航(已停)  │  lightning定位 + nav2导航    │
│  rl_deploy   │  rslidar/hsLidar│  ws_cxz + ws_nav            │
│  运动控制     │  雷达驱动        │  所有开发在这里              │
└─────────────┴─────────────────┴─────────────────────────────┘

数据流:
  106雷达 → /LIDAR/POINTS ──→ lightning定位(ws_cxz)
  103 IMU → /IMU ──────────→ lightning定位(ws_cxz)
  lightning → /lightning/odom + TF(map→lidar_link)
       ↓
  tf_bridge(ws_nav) → TF(map→odom→base_link→lidar_link)
       ↓
  nav2导航栈 → /cmd_vel
       ↓
  cmd_vel_to_navcmd(ws_nav) → /NAV_CMD → 103运动控制 → 机器狗行走
```

## 目录结构

| 路径 | 内容 |
|---|---|
| `/var/opt/robot/data/ws_cxz/` | lightning-lm 定位建图工作空间 |
| `/var/opt/robot/data/ws_cxz/src/lightning-lm-deep-robotics/` | lightning 源码 |
| `/var/opt/robot/data/ws_cxz/data/zjulib_backup/` | 建好的点云地图(global.pcd) |
| `/var/opt/robot/data/ws_nav/` | nav2 导航工作空间 |
| `/var/opt/robot/data/ws_nav/src/m20_nav/` | 导航包源码 |
| `/var/opt/robot/data/sdk_deploy/` | 旧版sdk（已弃用运动控制，保留teleop脚本）|

---

## 一、完整启动流程（3个终端）

### 前置准备（每次开机后执行一次）

```bash
# 1. 停止106上的原厂导航服务（避免冲突）
# 在104上执行：
ssh user@10.21.31.106    # 密码: '
su                        # 密码: '
systemctl stop handler.service
sleep 2
systemctl stop localization.service global_planner.service planner.service passable_area.service
exit
exit
```

或者用之前写的脚本（如果部署了）：
```bash
# 在104上（su后）
/var/opt/robot/data/sdk_deploy/stop_native_nav.sh stop
```

### 2. 终端1：启动 lightning 定位

```bash
ssh user@10.21.31.104        # 密码: '
cd /var/opt/robot/data/ws_cxz
source install/setup.bash
./src/lightning-lm-deep-robotics/sloc_rviz_session.sh
```

这会在 tmux 中打开多个窗口：
- 窗口0 'lightning': 定位程序（自动启动，绑核 cpu2-7 优先级90）
- 窗口1 'vis': rviz2 可视化
- 窗口2 'navstate': 定位状态
- 窗口3 'node_info': 命令行

**等待定位初始化完成**（看到 `imu init done` 和 `localization init success` 日志，约10-20秒）。

### 3. 终端2：启动 nav2 导航

```bash
ssh user@10.21.31.104
cd /var/opt/robot/data/ws_nav
source /opt/robot/scripts/setup_ros2.sh    # ← 关键！必须用这个，不能用 source /opt/ros/foxy/setup.bash
source install/setup.bash
ros2 launch m20_nav m20_navigation.launch.py 2>&1 | tee /var/opt/robot/data/ws_nav/nav_log.txt
```

**⚠️ 关键**：必须 `source /opt/robot/scripts/setup_ros2.sh`（它设置了 DDS 的 fastdds.xml 配置），否则 nav2 收不到 lightning 的定位数据。

等待所有节点 active（看到 `Managed nodes are active`）。

### 4. 终端3：rviz2 发送导航目标

在**笔记本电脑**上打开 rviz2（需要和机器狗在同一网段）：

```bash
# 笔记本上
source /opt/ros/foxy/setup.bash    # 或 humble，版本不重要
rviz2
```

rviz2 配置：
- **Fixed Frame**: `map`
- **Map display**: 话题选 `/map`（看静态地图）或 `/global_costmap/costmap`（看代价地图）
- **TF display**: 查看 map→odom→base_link 坐标链
- **Path display**: 话题选 `/plan`（导航路径）

发送导航目标：工具栏点击 **"2D Goal Pose"**，在地图上点击拖拽。

或者用命令行：
```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}"
```

### 5. 遥控器操作

- 机器狗遥控器切到**导航模式**
- 机器狗会自动走到目标点
- 空格/急停按钮随时可以停下

---

## 二、建图流程（如需重新建图）

### 1. 启动建图

```bash
cd /var/opt/robot/data/ws_cxz
source install/setup.bash
./src/lightning-lm-deep-robotics/start_lg_rviz_session.sh
```

（这和定位类似，但用的是 SLAM 模式的配置文件）

### 2. 遥控器控制机器狗走一圈

用遥控器控制机器狗慢速走遍整个区域。注意：
- 走慢一点（< 0.5 m/s），避免定位发散
- 尽量走闭环路径（回到起点），提高建图质量

### 3. 保存地图

```bash
# 在 tmux 的任意窗口执行
ros2 service call /lightning/save_map lightning/srv/SaveMap "{map_id: 'zjulib'}"
```

### 4. ⚠️ 立刻备份地图（重要！）

```bash
# Ctrl+C 退出建图前，必须先备份！
cp -r /var/opt/robot/data/ws_cxz/data/zjulib /var/opt/robot/data/ws_cxz/data/zjulib_backup_$(date +%Y%m%d)
```

**原因**：退出建图程序时会清空 data/zjulib/ 目录（SaveMap 的实现是先删后写），必须提前备份。

### 5. 更新定位配置指向新地图

```bash
# 修改定位配置的 map_path
vim /var/opt/robot/data/ws_cxz/src/lightning-lm-deep-robotics/config/default_deep_roboticsloc.yaml
# 把 map_path: ./data/zjulib_backup/ 改成你的新地图目录
```

同时更新 nav2 的地图路径：
```bash
vim /var/opt/robot/data/ws_nav/src/m20_nav/launch/m20_navigation.launch.py
# 修改 default_pcd 变量指向新的 global.pcd
```

---

## 三、键盘遥控（备用方案）

如果不想用 nav2 导航，可以手动遥控：

```bash
# 104上
source /opt/robot/scripts/setup_ros2.sh
python3 /var/opt/robot/data/sdk_deploy/nav_cmd_teleop.py
```

| 按键 | 功能 |
|---|---|
| W/S | 前进/后退 |
| A/D | 左移/右移 |
| Q/E | 左转/右转 |
| 空格 | 急停 |
| Ctrl+C | 退出（自动停车）|

速度参数在脚本头部修改（MAX_X_VEL 等）。

---

## 四、常用排查命令

### 检查定位是否正常
```bash
source /opt/robot/scripts/setup_ros2.sh
source /var/opt/robot/data/ws_cxz/install/setup.bash
# 看定位输出
ros2 topic echo /lightning/odom | head -20
# 看 TF
ros2 run tf2_ros tf2_echo map base_link
```

### 检查导航是否正常
```bash
source /opt/robot/scripts/setup_ros2.sh
source /var/opt/robot/data/ws_nav/install/setup.bash
# 看代价地图
ros2 topic echo /map | head -20
# 看 cmd_vel
ros2 topic echo /cmd_vel
# 看 NAV_CMD
ros2 topic echo /NAV_CMD
```

### 日志文件
| 文件 | 内容 |
|---|---|
| `/var/opt/robot/data/ws_cxz/loc_log.txt` | 定位日志 |
| `/var/opt/robot/data/ws_cxz/slam_log.txt` | 建图日志 |
| `/var/opt/robot/data/ws_nav/nav_log.txt` | 导航日志 |

---

## 五、关键注意事项

1. **启动顺序**：必须先启动 lightning 定位，等它初始化完成（~20秒），再启动 nav2 导航。

2. **DDS 配置**：所有节点（包括手动测试命令）都必须 `source /opt/robot/scripts/setup_ros2.sh`，不能用 `source /opt/ros/foxy/setup.bash`。否则 DDS 通信不通。

3. **106 服务**：每次机器狗重启后，106 上的原厂导航服务会自动恢复，需要重新停止。

4. **地图备份**：建图保存后立刻备份，否则退出建图程序时会被清空。

5. **速度安全**：nav2 的速度参数在 `/var/opt/robot/data/ws_nav/install/m20_nav/share/m20_nav/config/nav2_params.yaml` 中，max_vel_x 当前为 0.4 m/s（保守值）。

6. **遥控器急停**：任何时候遥控器的急停都有效，是最安全的后备。

7. **106 的 basic_server 不能停**：103 上的 basic_server 和 rl_deploy 是运动控制核心，绝对不能停。

---

## 六、重新编译（如需修改代码）

### lightning（定位建图）
```bash
cd /var/opt/robot/data/ws_cxz
source /opt/ros/foxy/setup.bash
colcon build --parallel-workers 3 --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release
# 约10分钟，用低内存模式避免OOM
```

### nav2（导航）
```bash
cd /var/opt/robot/data/ws_nav
source /opt/ros/foxy/setup.bash
colcon build --packages-select m20_nav
# Python包，很快
# ⚠️ 编译后需要修复可执行文件路径（foxy兼容性问题）
mkdir -p install/m20_nav/lib/m20_nav
cp install/m20_nav/bin/* install/m20_nav/lib/m20_nav/
chmod +x install/m20_nav/lib/m20_nav/*
```

---

## 七、系统时间问题

机器狗的系统时间会漂移（回到2024年），影响 apt 安装。如需安装软件：

```bash
# 在104上
su    # 密码: '
date -s '2026-XX-XX HH:MM:SS'    # 设为当前正确时间
hwclock --systohc                  # 写入硬件时钟
apt update                         # 现在可以正常安装了
```
