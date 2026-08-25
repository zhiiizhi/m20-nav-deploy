"""
 * @file m20_imu_bridge.py
 * @brief Bridge: /IMU_DATA (drdds/ImuData) → /imu (sensor_msgs/Imu)
 *        for FAST-LIO2 / RKO-LIO compatibility
"""

import rclpy
from rclpy.node import Node
from drdds.msg import ImuData
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
import math


class M20ImuBridge(Node):
    def __init__(self):
        super().__init__('m20_imu_bridge')

        self.sub = self.create_subscription(
            ImuData, '/IMU_DATA', self.imu_callback, 100)
        self.pub = self.create_publisher(Imu, '/IMU', 100)

        self.get_logger().info('IMU Bridge started: /IMU_DATA → /IMU')

    def imu_callback(self, msg):
        imu_out = Imu()
        h = Header()
        h.stamp = msg.header.stamp
        h.frame_id = 'base_link'
        imu_out.header = h

        r = math.radians(msg.data.roll)
        p = math.radians(msg.data.pitch)
        y = math.radians(msg.data.yaw)

        half_roll = r * 0.5
        half_pitch = p * 0.5
        half_yaw = y * 0.5
        cos_r = math.cos(half_roll)
        sin_r = math.sin(half_roll)
        cos_p = math.cos(half_pitch)
        sin_p = math.sin(half_pitch)
        cos_y = math.cos(half_yaw)
        sin_y = math.sin(half_yaw)

        imu_out.orientation.x = sin_r * cos_p * cos_y - cos_r * sin_p * sin_y
        imu_out.orientation.y = cos_r * sin_p * cos_y + sin_r * cos_p * sin_y
        imu_out.orientation.z = cos_r * cos_p * sin_y - sin_r * sin_p * cos_y
        imu_out.orientation.w = cos_r * cos_p * cos_y + sin_r * sin_p * sin_y

        imu_out.angular_velocity.x = float(msg.data.omega_x)
        imu_out.angular_velocity.y = float(msg.data.omega_y)
        imu_out.angular_velocity.z = float(msg.data.omega_z)

        imu_out.linear_acceleration.x = float(msg.data.acc_x)
        imu_out.linear_acceleration.y = float(msg.data.acc_y)
        imu_out.linear_acceleration.z = float(msg.data.acc_z)

        self.pub.publish(imu_out)


def main(args=None):
    rclpy.init(args=args)
    node = M20ImuBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
