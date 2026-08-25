#!/bin/bash
# Run M20 MuJoCo simulation with LiDAR SLAM (RKO-LIO + IMU Bridge)
# Usage: ./run_m20_slam.sh [with_rviz]
#
# This script opens 3 terminals:
#   Terminal 1: MuJoCo simulation (robot + LiDAR)
#   Terminal 2: IMU Bridge (/IMU_DATA → /imu)
#   Terminal 3: SLAM (RKO-LIO odometry)

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WORKSPACE_DIR="$(cd "$SCRIPT_DIR/../../.." && pwd)"
SIM_SCRIPT="$WORKSPACE_DIR/src/M20_sdk_deploy/interface/robot/simulation/mujoco_simulation_ros2.py"
BRIDGE_SCRIPT="$WORKSPACE_DIR/src/M20_sdk_deploy/interface/robot/simulation/m20_imu_bridge.py"
CONFIG_FILE="$WORKSPACE_DIR/src/M20_sdk_deploy/run_policy/fast_lio_config/m20_marsim.yaml"

WITH_RVIZ="${1:-false}"

# Source ROS2
source /opt/ros/humble/setup.bash
source "$WORKSPACE_DIR/install/setup.bash" 2>/dev/null || true

echo "========================================="
echo "  M20 Robot Dog - 3D LiDAR SLAM"
echo "========================================="
echo "  Simulation : $SIM_SCRIPT"
echo "  IMU Bridge : $BRIDGE_SCRIPT"
echo "  SLAM Config: $CONFIG_FILE"
echo "  RVIZ       : $WITH_RVIZ"
echo "========================================="

cleanup() {
    echo ""
    echo "Shutting down..."
    kill $PID_SIM $PID_BRIDGE $PID_SLAM $PID_TFRELAY 2>/dev/null
    wait $PID_SIM $PID_BRIDGE $PID_SLAM $PID_TFRELAY 2>/dev/null
    echo "All stopped."
}
trap cleanup EXIT INT TERM

# Terminal 1: Start MuJoCo simulation
echo "[1/4] Starting MuJoCo simulation..."
python3 "$SIM_SCRIPT" &
PID_SIM=$!
sleep 3

# Terminal 2: Start IMU Bridge
echo "[2/4] Starting IMU Bridge..."
ros2 run rclpy_components standalone_node "$BRIDGE_SCRIPT" 2>/dev/null || \
python3 "$BRIDGE_SCRIPT" &
PID_BRIDGE=$!
sleep 1

# Terminal 3: Start SLAM
echo "[3/4] Starting RKO-LIO SLAM..."
RVIZ_ARG=""
if [ "$WITH_RVIZ" = "true" ]; then
    RVIZ_ARG="rviz:=true"
fi

ros2 launch rko_lio odometry.launch.py \
    config_file:="$CONFIG_FILE" \
    mode:=online \
    $RVIZ_ARG &
PID_SLAM=$!
sleep 2

# Terminal 4: Odometry → TF relay
echo "[4/4] Starting odom→TF relay..."
python3 "$WORKSPACE_DIR/src/M20_sdk_deploy/interface/robot/simulation/odom_to_tf.py" &
PID_TFRELAY=$!

# Wait for all background processes
echo ""
echo "========================================="
echo "  所有节点已启动！"
echo "  RViz: rviz2 -d <SDK>/src/M20_sdk_deploy/run_policy/fast_lio_config/m20_slam.rviz"
echo "  Fixed Frame: odom"
echo "  按 Ctrl+C 停止所有节点"
echo "========================================="
echo ""

wait
