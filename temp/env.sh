#!/bin/bash
# sdk_deploy 笔记本环境初始化脚本
# 用法: source ~/sdk_deploy/env.sh
#
# 替代原来的 source install/setup.bash，额外补全了台式机 -> 笔记本
# 迁移后缺失的库路径 (onnxruntime, glog, pangolin)。

SCRIPT_DIR="$(builtin cd "$(dirname "${BASH_SOURCE[0]}")" > /dev/null && pwd)"

# 1. 加载 ROS2 Humble
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
else
    echo "[ERROR] ROS2 Humble not found at /opt/ros/humble" 1>&2
    return 1
fi

# 2. 加载项目 install 环境
if [ -f "$SCRIPT_DIR/install/local_setup.bash" ]; then
    source "$SCRIPT_DIR/install/local_setup.bash"
else
    echo "[ERROR] Project install not found at $SCRIPT_DIR/install" 1>&2
    return 1
fi

# 3. 补全 LD_LIBRARY_PATH (台式机编译产物 RPATH 指向旧路径)
# onnxruntime  -> rl_deploy 需要
export LD_LIBRARY_PATH="$SCRIPT_DIR/src/M20_sdk_deploy/third_party/onnxruntime/x86/lib:$LD_LIBRARY_PATH"
# glog .so.1   -> lightning/run_loc_online 需要 (系统只有 libglog.so.0)
export LD_LIBRARY_PATH="$SCRIPT_DIR/src/lightning-lm/thirdparty/glog/build:$LD_LIBRARY_PATH"
# Pangolin     -> lightning/run_loc_online 需要
export LD_LIBRARY_PATH="$SCRIPT_DIR/src/lightning-lm/thirdparty/Pangolin/build:$LD_LIBRARY_PATH"

echo "[env] sdk_deploy ready. ROS_DISTRO=$ROS_DISTRO"
