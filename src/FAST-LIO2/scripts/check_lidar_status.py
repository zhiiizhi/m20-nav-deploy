#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import time
import rospy
import rosgraph
import argparse
from std_srvs.srv import Trigger
from mavros_msgs.msg import State
from mavros_msgs.msg import RCIn

# 导入消息类型：确保已安装 livox_ros_driver2 和相关依赖（nav_msgs 等）
from livox_ros_driver2.msg import CustomMsg
from nav_msgs.msg import Odometry

def wait_for_master(timeout=10.0):
    """
    等待 roscore 是否在线，在超时时间内循环检测。
    如果在 timeout 内检测到 roscore，上报 True，否则 False。
    """
    start_time = time.time()
    while (time.time() - start_time) < timeout:
        if rosgraph.is_master_online():
            return True
        else:
            print("rosmaster is not good, wait...")
            time.sleep(0.1)
    return False

def wait_for_lidar(topic, timeout):
    print(f"Waiting for lidar message at {topic}...")
    # 2. 检测 Lidar 话题是否收到消息
    try:
        msg_lidar = rospy.wait_for_message(
            topic, 
            CustomMsg, 
            timeout=timeout
        )
    except rospy.ROSException:
        print(f"No messages received on topic '{topic}' within {timeout} seconds.")
        return False
    if msg_lidar.point_num == 0:
        print("Lidar point_num is 0.")
        return False
    print (f"point_num is {msg_lidar.point_num}")
    return True

def callMapSaveService(service_name="/save_lio_pcl"):
    # Call the service, which type is std_srvs/Trigger
    print(f"Calling service {service_name}...")
    rospy.wait_for_service(service_name)
    try:
        save_map = rospy.ServiceProxy(service_name, Trigger)
        resp = save_map()
        print(f"Map saved to: {resp.message}")
        return resp.success
    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")
        return False
    
class MavStatus:
    def __init__(self, ns=""):
        self.armed = False
        self.killed = False
        self.sub = rospy.Subscriber(
            ns + "/mavros/state", 
            State, 
            self.handleMavState
        )

    def handleMavState(self, state):
        # Check if armed
        if self.armed != state.armed:
            self.armed = state.armed
            if self.armed:
                print("Vehicle is armed.")
            else:
                print("Vehicle is disarmed. Will savemap")
                callMapSaveService()

def main():
    # 通过 argparse 解析命令行参数
    parser = argparse.ArgumentParser(
        description="Check roscore, Lidar topic, point_num, and Odom topic."
    )
    parser.add_argument(
        "--lidar",
        type=str,
        default="/livox/lidar",
        help="Topic name for Lidar messages (default: /livox/lidar)"
    )
    parser.add_argument(
        "--odom",
        type=str,
        default="/Odometry",
        help="Topic name for Odom messages (default: /Odometry)"
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=10.0,
        help="Timeout in seconds for each check (default: 10.0)"
    )
    
    # only-lidar will only check if lidar message is available and exit
    parser.add_argument(
        "--only-lidar",
        action="store_true",
    )

    args = parser.parse_args()

    # 1. 检测 roscore 是否在线
    if not wait_for_master(args.timeout):
        print("No roscore detected within timeout.")
        sys.exit(-1)

    # 初始化 ROS 节点
    rospy.init_node("check_point_num", anonymous=True)

    succ = wait_for_lidar(args.lidar, args.timeout)
    
    if not succ:
        sys.exit(-1)
    elif args.only_lidar:
        print("All checks for lidar is passed.")
        sys.exit(0)
        
    mav = MavStatus()
    rate = rospy.Rate(1) # 1hz
    # 4. 检测 Odom 话题是否收到消息
    while not rospy.is_shutdown():
        print(f"Waiting for Odometry message at {args.odom}...")
        try:
            msg_odom = rospy.wait_for_message(
                args.odom,
                Odometry, 
                timeout=args.timeout
            )
        except rospy.ROSException:
            print(f"No messages received on topic '{args.odom}' within {args.timeout} seconds.")
            print("Will save map and reboot the LIO in 3 seconds.")
            callMapSaveService()
            time.sleep(3)
            sys.exit(-1)
        print (f"Odom message received: {msg_odom.pose.pose}")
        rate.sleep()

    sys.exit(-1)

if __name__ == "__main__":
    main()
