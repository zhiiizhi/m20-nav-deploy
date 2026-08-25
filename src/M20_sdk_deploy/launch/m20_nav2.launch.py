#!/usr/bin/env python3
"""
M20 Nav2 导航启动文件

启动顺序（4 个终端）：
1. MuJoCo 仿真
2. IMU bridge
3. Lightning-LM 定位（手动启动）
4. 运行此启动文件（启动 TF bridge + 点云转换 + Nav2）

使用方法：
  cd ~/sdk_deploy && source install/setup.bash
  # 终端 1: 启动仿真和 IMU bridge
  python3 src/M20_sdk_deploy/interface/robot/simulation/mujoco_simulation_ros2.py &
  python3 src/M20_sdk_deploy/interface/robot/simulation/m20_imu_bridge.py &
  sleep 5
  # 终端 2: 启动 Lightning-LM 定位
  ros2 run lightning run_loc_online --config src/lightning-lm/config/m20_sim_slam.yaml
  # 终端 3: 启动 RL control（可选，用于键盘控制 fallback）
  ros2 run m20_sdk_deploy rl_deploy
  # 终端 4: 启动导航栈
  ros2 launch m20_sdk_deploy m20_nav2.launch.py
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    nav2_pkg_share = FindPackageShare('nav2_bringup').find('nav2_bringup')
    m20_pkg_share = get_package_share_directory('m20_sdk_deploy')
    nav2_params = os.path.join(m20_pkg_share, 'config', 'nav2', 'nav2_params.yaml')

    return LaunchDescription([
        # TF bridge: creates odom->base_link from Lightning-LM odometry
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_lidar_link',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'lidar_link'],
            output='screen',
        ),
        Node(
            package='m20_sdk_deploy',
            executable='lightning_tf_bridge.py',
            name='lightning_tf_bridge',
            output='screen',
        ),

        # 点云→栅格地图转换节点
        Node(
            package='m20_sdk_deploy',
            executable='pcd_to_costmap.py',
            name='pcd_to_costmap',
            output='screen',
            parameters=[{
                'map_path': '/workspace/sdk_deploy/data/office4f/global.pcd',
                'resolution': 0.05,
                'map_width': 50.0,
                'map_height': 50.0,
                'z_ground_threshold': 0.1,
                'z_max': 2.0,
                'robot_radius': 0.35,
                'inflation_radius': 0.55,
            }]
        ),

        # Nav2 导航栈
        IncludeLaunchDescription(
            PathJoinSubstitution([nav2_pkg_share, 'launch', 'navigation_launch.py']),
            launch_arguments={
                'use_sim_time': 'False',
                'params_file': nav2_params,
                'autostart': 'True',
            }.items()
        ),
    ])
