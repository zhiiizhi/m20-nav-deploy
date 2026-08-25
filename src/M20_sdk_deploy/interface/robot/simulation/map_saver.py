import argparse
import sys
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


class MapSaver(Node):
    def __init__(self, output_path: str):
        super().__init__('map_saver')
        self.output_path = output_path
        self.points: list[np.ndarray] = []

        self.sub = self.create_subscription(
            PointCloud2, '/rko_lio/local_map', self.cb, 10)
        self.get_logger().info(
            f'MapSaver: listening to /rko_lio/local_map, will save to {output_path} on shutdown')

    def cb(self, msg):
        try:
            frame = point_cloud2.read_points_numpy(msg, field_names=['x', 'y', 'z'])
            if len(frame) == 0:
                return
            self.points.append(frame)
            self.get_logger().info(
                f'Received frame: {len(frame)} pts, total: {sum(len(p) for p in self.points)}',
                throttle_duration_sec=5.0)
        except Exception as e:
            self.get_logger().warn(f'Failed to read point cloud: {e}')

    def save(self):
        if not self.points:
            self.get_logger().warn('No points collected, nothing to save.')
            return

        all_pts = np.concatenate(self.points, axis=0)
        self.get_logger().info(f'Total raw points: {len(all_pts)}')

        # simple voxel filter (~10cm grid)
        if len(all_pts) > 0:
            voxel_size = 0.1
            voxel_indices = np.floor(all_pts / voxel_size).astype(np.int64)
            _, unique_idx = np.unique(voxel_indices, axis=0, return_index=True)
            all_pts = all_pts[np.sort(unique_idx)]
            self.get_logger().info(f'After voxel filter ({voxel_size}m): {len(all_pts)} points')

        self._write_pcd(all_pts)
        self.get_logger().info(f'Map saved to {self.output_path}')

    def _write_pcd(self, pts: np.ndarray):
        n = len(pts)
        with open(self.output_path, 'w') as f:
            f.write(
                '# .PCD v0.7 - Point Cloud Data file format\n'
                'VERSION 0.7\n'
                'FIELDS x y z\n'
                'SIZE 4 4 4\n'
                'TYPE F F F\n'
                'COUNT 1 1 1\n'
                f'WIDTH {n}\n'
                'HEIGHT 1\n'
                'VIEWPOINT 0 0 0 1 0 0 0\n'
                f'POINTS {n}\n'
                'DATA ascii\n'
            )
            for p in pts:
                f.write(f'{p[0]:.6f} {p[1]:.6f} {p[2]:.6f}\n')


def main():
    parser = argparse.ArgumentParser(description='Save RKO-LIO local map as PCD')
    parser.add_argument('--output', '-o', default='m20_map.pcd',
                        help='Output .pcd file path')
    args = parser.parse_args()

    rclpy.init()
    node = MapSaver(args.output)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
