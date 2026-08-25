#!/usr/bin/env python3
"""
M20 定位/导航可视化 launch
========================
仅启动 rviz2，加载预设配置，用于查看你的 lightning 定位 + nav2 导航效果。

它不启动任何算法节点。你需要按顺序在其它终端先起好：
  1. lightning 定位:   ros2 run lightning run_loc_online --config .../default_deep_roboticsloc.yaml
  2. nav2 导航栈:      ros2 launch m20_sdk_deploy m20_nav2.launch.py
然后再起这个 rviz。

用法:
  ros2 launch m20_sdk_deploy m20_rviz.launch.py

rviz 里能看到什么:
  - GlobalMap     /lightning/global_map       建图存的全局点云(灰白背景)
  - CurrentScan   /lightning/current_scan_cloud  当前激光扫描(红色, 实时定位)
  - LightningPath /lightning/path             定位轨迹(绿线)
  - NavMap        /map                        pcd_to_costmap 的栅格地图
  - NavPlan       /plan                       nav2 全局路径(蓝线)
  - LocalPlan     /local_plan                 局部路径(黄线)
  - TF            map->odom->base_link        坐标链

发导航目标:
  点顶部工具栏 "2D Goal Pose", 在地图上拖拽 -> 发到 /goal_pose (nav2 自动执行)
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    m20_pkg_share = get_package_share_directory('m20_sdk_deploy')
    default_rviz = os.path.join(m20_pkg_share, 'config', 'm20_nav_view.rviz')

    rviz_config = LaunchConfiguration('rviz_config')

    return LaunchDescription([
        DeclareLaunchArgument(
            'rviz_config',
            default_value=default_rviz,
            description='rviz 配置文件路径'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='m20_rviz',
            output='screen',
            arguments=['-d', rviz_config],
        ),
    ])
