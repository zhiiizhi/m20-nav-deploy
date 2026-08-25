#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import open3d as o3d
import numpy as np
import matplotlib
import matplotlib.colors as mcolors

def colorize_by_height_colormap(pcd: o3d.geometry.PointCloud, cmap_name="viridis"):
    points = np.asarray(pcd.points)
    z_vals = points[:, 2]
    norm = mcolors.Normalize(vmin=z_vals.min(), vmax=z_vals.max())
    cmap = matplotlib.colormaps.get_cmap(cmap_name)
    rgba_vals = cmap(norm(z_vals))  # Nx4
    rgb_vals = rgba_vals[:, :3]
    pcd.colors = o3d.utility.Vector3dVector(rgb_vals)
    return pcd

def main():
    parser = argparse.ArgumentParser(description='读取并可视化 PCD 文件，并按高度着色')
    parser.add_argument('pcd_file', type=str, help='PCD 文件路径')
    parser.add_argument('--cmap', type=str, default='viridis', help='colormap 名称 (默认: viridis)')
    args = parser.parse_args()

    # 读取点云
    pcd = o3d.io.read_point_cloud(args.pcd_file)
    print(f"成功读取点云：{args.pcd_file}, 点数量：{len(pcd.points)}")

    # 进行颜色映射
    pcd = colorize_by_height_colormap(pcd, cmap_name=args.cmap)

    # 可视化
    o3d.visualization.draw_geometries([pcd])

if __name__ == '__main__':
    main()
