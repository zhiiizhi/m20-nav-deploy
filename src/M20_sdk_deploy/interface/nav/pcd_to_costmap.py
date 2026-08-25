#!/usr/bin/env python3
"""
Convert a PCD pointcloud map to 2D occupancy grid for Nav2.

This node loads a PCD map file from disk (typically created by Lightning-LM SLAM),
projects it onto a 2D XY grid, and publishes it as a nav_msgs/OccupancyGrid
on the /map topic with TRANSIENT_LOCAL durability.

Usage:
  ros2 run m20_sdk_deploy pcd_to_costmap.py --ros-args -p map_path:=/home/cxz/sdk_deploy/data/m20_sim_map/global.pcd
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
import numpy as np
import open3d as o3d


class PCDToCostmap(Node):
    def __init__(self):
        super().__init__('pcd_to_costmap')

        # Parameters
        self.declare_parameter('map_path', '/home/admi/sdk_deploy/data/m20_sim_map/global.pcd')
        self.declare_parameter('resolution', 0.05)
        self.declare_parameter('z_ground_threshold', 0.1)
        self.declare_parameter('z_max', 2.0)
        self.declare_parameter('map_padding', 20.0)
        self.declare_parameter('inflation_radius', 0.55)

        self.map_path = self.get_parameter('map_path').value
        self.resolution = self.get_parameter('resolution').value
        self.z_ground_threshold = self.get_parameter('z_ground_threshold').value
        self.z_max = self.get_parameter('z_max').value
        self.padding = self.get_parameter('map_padding').value
        self.inflation_radius = self.get_parameter('inflation_radius').value

        map_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.VOLATILE,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self.map_pub = self.create_publisher(
            OccupancyGrid,
            '/map',
            map_qos
        )

        # 首次加载 + 每 3 秒重发（解决 Foxy static_layer VOLATILE 订阅的时序问题）
        self.load_and_publish_map()
        self.create_timer(3.0, self.load_and_publish_map)

    def load_and_publish_map(self):
        """Load PCD file using Open3D and publish occupancy grid."""
        self.get_logger().info(f"Loading map from: {self.map_path}")
        pcd = o3d.io.read_point_cloud(self.map_path)
        points = np.asarray(pcd.points)

        if len(points) == 0:
            self.get_logger().error("Failed to load PCD map or no points found")
            return

        self.get_logger().info(f"Loaded {len(points)} points from PCD")

        # Filter by height
        zs = points[:, 2]
        z_mask = (zs > self.z_ground_threshold) & (zs < self.z_max)
        filtered = points[z_mask]

        if len(filtered) == 0:
            self.get_logger().warn("No points in height range after filtering")
            return

        # Compute map bounds from pointcloud
        min_x = filtered[:, 0].min() - self.padding
        max_x = filtered[:, 0].max() + self.padding
        min_y = filtered[:, 1].min() - self.padding
        max_y = filtered[:, 1].max() + self.padding

        map_width_m = max_x - min_x
        map_height_m = max_y - min_y

        map_width_cells = int(map_width_m / self.resolution)
        map_height_cells = int(map_height_m / self.resolution)

        self.get_logger().info(
            f"Map bounds: x=[{min_x:.1f}, {max_x:.1f}], y=[{min_y:.1f}, {max_y:.1f}], "
            f"size={map_width_cells}x{map_height_cells} cells, "
            f"{len(filtered)} obstacle points"
        )

        # Create occupancy grid
        grid_data = np.zeros(map_height_cells * map_width_cells, dtype=np.int8)

        # Project points to grid
        grid_xs = ((filtered[:, 0] - min_x) / self.resolution).astype(np.int32)
        grid_ys = ((filtered[:, 1] - min_y) / self.resolution).astype(np.int32)

        # Clamp to bounds
        grid_xs = np.clip(grid_xs, 0, map_width_cells - 1)
        grid_ys = np.clip(grid_ys, 0, map_height_cells - 1)

        grid_indices = grid_ys * map_width_cells + grid_xs
        grid_data[grid_indices] = 100  # occupied

        # Inflate obstacles
        inflation_cells = int(self.inflation_radius / self.resolution)
        if inflation_cells > 0:
            grid_2d = grid_data.reshape((map_height_cells, map_width_cells))
            inflated = np.zeros_like(grid_2d)
            for dy in range(-inflation_cells, inflation_cells + 1):
                for dx in range(-inflation_cells, inflation_cells + 1):
                    dist = np.sqrt(dx*dx + dy*dy) * self.resolution
                    if dist <= self.inflation_radius:
                        shifted = np.roll(np.roll(grid_2d, dy, axis=0), dx, axis=1)
                        inflated = np.maximum(inflated, shifted)
            grid_data = inflated.flatten()

        # Publish occupancy grid
        occupancy_msg = OccupancyGrid()
        occupancy_msg.header.stamp = self.get_clock().now().to_msg()
        occupancy_msg.header.frame_id = 'map'
        occupancy_msg.info.resolution = self.resolution
        occupancy_msg.info.width = map_width_cells
        occupancy_msg.info.height = map_height_cells
        occupancy_msg.info.origin.position.x = min_x
        occupancy_msg.info.origin.position.y = min_y
        occupancy_msg.info.origin.position.z = 0.0
        occupancy_msg.info.origin.orientation.w = 1.0
        occupancy_msg.data = grid_data.tolist()

        self.map_pub.publish(occupancy_msg)
        self.get_logger().info(
            f"Published occupancy grid: {map_width_cells}x{map_height_cells} cells, "
            f"resolution={self.resolution}m, origin=({min_x:.1f}, {min_y:.1f})"
        )


def main(args=None):
    rclpy.init(args=args)
    node = PCDToCostmap()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
