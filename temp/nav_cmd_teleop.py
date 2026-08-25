#!/usr/bin/env python3
"""
M20 键盘遥控 - 通过 /NAV_CMD 控制机器狗（云深处官方运动控制）
================================================================
不需要运行自己的 rl_deploy！机器狗 103 主机上的 basic_server + rl_deploy
（云深处官方运动控制）一直在运行，本脚本直接发布 /NAV_CMD 速度指令即可。

前置条件：
  1. 用遥控器将机器狗切换到「导航模式」
  2. 关闭 106 的 planner 服务（避免冲突）：systemctl stop planner.service
  3. 确认 103 的 basic_server 在运行（默认开机自启）

使用方法（在 104 或任意能访问 ROS 网络的主机上）：
  source /opt/ros/foxy/setup.bash    # 或 humble
  python3 nav_cmd_teleop.py

操作说明：
  W/S : 前进/后退
  A/D : 左移/右移
  Q/E : 左转/右转（逆时针/顺时针）
  空格: 急停（速度归零）
  R   : 切换到 RL 控制状态（state=17，需先用遥控器进导航模式）
  按住键持续运动，松开自动减速归零
  Ctrl+C 退出

注意：/NAV_CMD 建议 10Hz 发布，本脚本默认 10Hz。
"""

import sys
import os
import time
import termios
import tty
import threading

import rclpy
from rclpy.node import Node

# drdds 消息（云深处自定义）
try:
    from drdds.msg import NavCmd as NavCmdMsg
    from drdds.msg import MotionState
    from drdds.msg import Gait as GaitMsg
except ImportError:
    print("[ERROR] 无法导入 drdds 消息类型！")
    print("请先 source ROS 环境: source /opt/ros/foxy/setup.bash")
    print("并确认 drdds 包已安装（机器狗上自带）")
    sys.exit(1)


# ============= 速度参数（按需调整）=============
MAX_X_VEL = 1.2    # 前进/后退最大速度 m/s（M20 建议 0.3~0.6）
MAX_Y_VEL = 0.4    # 左右平移最大速度 m/s（轮足可侧移，比 sdk 模式更稳）
MAX_YAW_VEL = 0.8  # 转向最大角速度 rad/s
ACCEL = 0.15       # 每帧加速量（按键按下时速度爬升）
DECEL = 0.19       # 每帧减速量（松开时速度归零）


class NavCmdTeleop(Node):
    def __init__(self):
        super().__init__('nav_cmd_teleop')

        self.nav_pub = self.create_publisher(NavCmdMsg, '/NAV_CMD', 10)
        self.motion_pub = self.create_publisher(MotionState, '/MOTION_STATE', 10)

        # 当前速度（平滑过渡）
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_yaw = 0.0
        self.cur_x = 0.0
        self.cur_y = 0.0
        self.cur_yaw = 0.0

        # 当前按下的键
        self.keys_held = set()

        # 10Hz 定时发布
        self.frame_id = 0
        self.timer = self.create_timer(0.1, self.timer_cb)  # 10Hz

        self.get_logger().info(
            '\n╔══════════════════════════════════════════════╗\n'
            '║   M20 键盘遥控 (/NAV_CMD 官方运动控制)       ║\n'
            '╚══════════════════════════════════════════════╝\n'
            '  前提: 遥控器切到「导航模式」\n'
            '  W/S : 前进/后退    A/D : 左移/右移\n'
            '  Q/E : 左转/右转    空格 : 急停\n'
            '  按住键持续运动，松开自动归零\n'
            '  Ctrl+C 退出\n')

    def smooth(self, cur, target, dt):
        """平滑加速/减速"""
        if target > cur:
            return min(target, cur + ACCEL)
        elif target < cur:
            return max(target, cur - DECEL)
        return cur

    def timer_cb(self):
        """10Hz 发布 /NAV_CMD"""
        # 根据按键计算目标速度
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_yaw = 0.0

        if 'w' in self.keys_held:
            self.target_x += MAX_X_VEL
        if 's' in self.keys_held:
            self.target_x -= MAX_X_VEL
        if 'a' in self.keys_held:
            self.target_y += MAX_Y_VEL
        if 'd' in self.keys_held:
            self.target_y -= MAX_Y_VEL
        if 'q' in self.keys_held:
            self.target_yaw += MAX_YAW_VEL
        if 'e' in self.keys_held:
            self.target_yaw -= MAX_YAW_VEL

        # 平滑过渡
        self.cur_x = self.smooth(self.cur_x, self.target_x, 0.1)
        self.cur_y = self.smooth(self.cur_y, self.target_y, 0.1)
        self.cur_yaw = self.smooth(self.cur_yaw, self.target_yaw, 0.1)

        # 发布（MetaType 的字段: frame_id + stamp[builtin_interfaces/Time: sec/nanosec]）
        msg = NavCmdMsg()
        msg.header.frame_id = self.frame_id
        self.frame_id += 1
        now = self.get_clock().now().to_msg()
        msg.header.stamp.sec = now.sec
        msg.header.stamp.nanosec = now.nanosec
        msg.data.x_vel = self.cur_x
        msg.data.y_vel = self.cur_y
        msg.data.yaw_vel = self.cur_yaw
        self.nav_pub.publish(msg)

    def send_motion_state(self, state):
        """切换运动状态（如 RL控制=17）"""
        msg = MotionState()
        msg.header.frame_id = self.frame_id
        now = self.get_clock().now().to_msg()
        msg.header.stamp.sec = now.sec
        msg.header.stamp.nanosec = now.nanosec
        msg.data.state = state
        self.motion_pub.publish(msg)
        state_names = {1: '站立', 2: '软急停', 4: '趴下', 17: 'RL控制'}
        self.get_logger().info(f'[MOTION_STATE] 切换到: {state} ({state_names.get(state, "未知")})')


def keyboard_listener(node):
    """非阻塞读取键盘输入，用 select 实现可靠的按键/松键检测。

    机制：每 50ms 检查一次 stdin。按键事件记录到 last_seen[k]=当前时间；
    超过 150ms 没收到某键，视为松开，从 keys_held 移除。
    """
    import select
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    last_seen = {}
    try:
        tty.setraw(fd)
        while rclpy.ok():
            # 非阻塞等待 50ms
            r, _, _ = select.select([sys.stdin], [], [], 0.05)
            now = time.time()
            if r:
                ch = sys.stdin.read(1).lower()
                if ch == '\x03':  # Ctrl+C
                    break
                elif ch == ' ':  # 空格 = 急停
                    node.keys_held.clear()
                    node.cur_x = node.cur_y = node.cur_yaw = 0.0
                    print('\r[急停] 速度归零          ', end='', flush=True)
                elif ch == 'r':  # 切换到 RL 控制
                    node.send_motion_state(17)
                elif ch in ('w', 'a', 's', 'd', 'q', 'e'):
                    node.keys_held.add(ch)
                    last_seen[ch] = now
                    print(f'\r[按键] {sorted(node.keys_held)}  '
                          f'v=({node.cur_x:+.2f},{node.cur_y:+.2f},{node.cur_yaw:+.2f})   ',
                          end='', flush=True)
            # 清理超时的按键（松开检测）
            for k in list(node.keys_held):
                if now - last_seen.get(k, 0) > 0.15:
                    node.keys_held.discard(k)
                    if not node.keys_held:
                        print('\r[松开] 减速中...          ', end='', flush=True)
    except Exception:
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


def main():
    rclpy.init()
    node = NavCmdTeleop()

    # 键盘监听线程
    kb_thread = threading.Thread(target=keyboard_listener, args=(node,), daemon=True)
    kb_thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 退出前急停
        msg = NavCmdMsg()
        msg.header.frame_id = node.frame_id
        now = node.get_clock().now().to_msg()
        msg.header.stamp.sec = now.sec
        msg.header.stamp.nanosec = now.nanosec
        msg.data.x_vel = 0.0
        msg.data.y_vel = 0.0
        msg.data.yaw_vel = 0.0
        node.nav_pub.publish(msg)
        print('\n[退出] 已发送急停指令')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
