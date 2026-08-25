#!/usr/bin/env python3
"""
Subscribe to Lightning-LM odometry and publish TF odom -> base_link.

Lightning-LM publishes /lightning/odom with:
  header.frame_id = "map"
  child_frame_id = "lidar_link"

This node creates:
  odom -> base_link  (using odometry pose)

Combined with Lightning-LM's map -> lidar_link, the full chain is:
  map -> lidar_link (Lightning-LM)
  odom -> base_link (this node)

For Nav2 to work, we also need map -> odom. Since Lightning-LM publishes
map -> lidar_link directly, we create a static transform odom -> map inverse
or simply have this node publish odom -> base_link with frame_id="odom".

Actually, the simplest approach: publish odom -> base_link directly from odometry,
and Nav2 will use the map -> odom from Lightning-LM (if it publishes it) or
we create a static odom -> map transform.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


class LightningOdomToTF(Node):
    def __init__(self):
        super().__init__('lightning_odom_to_tf')
        self.sub = self.create_subscription(
            Odometry, '/lightning/odom', self.cb, 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.get_logger().info(
            'LightningOdomToTF: /lightning/odom -> TF odom->base_link')

    def cb(self, msg):
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = LightningOdomToTF()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
