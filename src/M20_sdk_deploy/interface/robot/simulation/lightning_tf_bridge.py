#!/usr/bin/env python3
"""
Lightning-LM 纯定位 TF Bridge（不依赖 /ODOM）
=============================================
原版 lightning_tf_bridge.py 依赖机器狗原生 /ODOM（足式里程计）来发 odom->base_link，
并用 lightning 的 map->lidar_link 算 map->odom 校正。

当你关闭厂家 localization 服务（/ODOM 没了）时，原版 bridge 无法工作，nav2 会报
"Invalid frame ID odom" / "base_link to odom timed out"。

本节点改为纯用 lightning 自己发布的 /lightning/odom：
  - /lightning/odom 的 header.frame_id = "map", child_frame_id = "lidar_link"
  - 它携带的是 lightning 在 map 坐标系下的完整位姿（PGO 输出）

TF 输出（补全 nav2 需要的链）:
  map      -> odom        （恒等变换，让 odom 与 map 重合）
  odom     -> base_link   （用 /lightning/odom 的位姿）
  base_link-> lidar_link  （静态；IMU-LiDAR外参目前为0，故重合）

这样 nav2 的 map -> odom -> base_link 链完整可用，完全不依赖 /ODOM。
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster


class LightningPureTFBridge(Node):
    def __init__(self):
        super().__init__('lightning_tf_bridge')

        # 订阅 lightning 自己发的里程计（map 坐标系下的位姿）
        self.sub = self.create_subscription(
            Odometry, '/lightning/odom', self.odom_cb, 10)

        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_broadcaster = StaticTransformBroadcaster(self)

        # 发布静态变换
        self.publish_static_transforms()
        # 定时重发，防止晚启动的订阅者错过
        self.create_timer(5.0, self.republish_static)

        self.get_logger().info(
            'LightningPureTFBridge: 纯用 /lightning/odom 发 TF '
            '(map->odom->base_link)，不依赖 /ODOM')

    def publish_static_transforms(self):
        now = self.get_clock().now().to_msg()

        # 注意：不发 base_link->lidar_link！
        # lightning 自己会发 map->lidar_link，如果 bridge 再连 base_link->lidar_link，
        # 会导致 lidar_link 有两个父节点(map 和 base_link)，TF 报 "two unconnected trees"。
        # 所以 bridge 只负责 map->odom->base_link 这条链，lidar_link 让 lightning 管。

        # map -> odom（静态恒等，让 odom 与 map 重合）
        t2 = TransformStamped()
        t2.header.stamp = now
        t2.header.frame_id = 'map'
        t2.child_frame_id = 'odom'
        t2.transform.rotation.w = 1.0
        self.static_broadcaster.sendTransform(t2)

    def republish_static(self):
        """定时重发 map->odom，防止晚启动的订阅者错过 TRANSIENT_LOCAL 消息。"""
        self.publish_static_transforms()

    def odom_cb(self, msg):
        # /lightning/odom 的位姿是 map 系下的，直接用作 odom->base_link
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        # foxy 兼容：必须逐字段赋值，不能整体赋 position/orientation 对象
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation.x = msg.pose.pose.orientation.x
        t.transform.rotation.y = msg.pose.pose.orientation.y
        t.transform.rotation.z = msg.pose.pose.orientation.z
        t.transform.rotation.w = msg.pose.pose.orientation.w
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = LightningPureTFBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
