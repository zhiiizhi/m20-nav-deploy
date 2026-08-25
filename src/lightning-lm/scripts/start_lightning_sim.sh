#!/bin/bash
# Lightning-LM + M20 Simulation one-click launch
# Usage: ./start_lightning_sim.sh

set -e

WORKSPACE=~/sdk_deploy
cd "$WORKSPACE"
source install/setup.bash

echo "============================================="
echo " Lightning-LM + M20 Simulation"
echo "============================================="
echo ""
echo "Topics:"
echo "  LiDAR: /LIDAR/POINTS (merged, ~9Hz)"
echo "  IMU:   /IMU"
echo ""
echo "WARNING: Keep the robot STATIONARY for 5 seconds after start!"
echo ""

# Cleanup any previous runs
pkill -f mujoco_simulation 2>/dev/null || true
pkill -f m20_imu_bridge 2>/dev/null || true
pkill -f run_slam_online 2>/dev/null || true
sleep 1

echo "[1/3] Starting M20 Simulation..."
python3 src/M20_sdk_deploy/interface/robot/simulation/mujoco_simulation_ros2.py &
SIM_PID=$!
sleep 2

echo "[2/3] Starting IMU Bridge..."
python3 src/M20_sdk_deploy/interface/robot/simulation/m20_imu_bridge.py &
IMU_PID=$!
sleep 1

# Verify topics
echo "[3/3] Verifying topics..."
sleep 2
LIDAR_HZ=$(ros2 topic hz /LIDAR/POINTS --window 5 2>&1 | grep "average" | awk '{print $3}' || echo "N/A")
IMU_HZ=$(ros2 topic hz /IMU --window 5 2>&1 | grep "average" | awk '{print $3}' || echo "N/A")
echo "  /LIDAR/POINTS: ~${LIDAR_HZ} Hz"
echo "  /IMU: ~${IMU_HZ} Hz"

echo ""
echo "============================================="
echo " Simulation + IMU Bridge running"
echo " PID: sim=$SIM_PID imu=$IMU_PID"
echo "============================================="
echo ""
echo "NOW LAUNCH LIGHTNING-LM IN ANOTHER TERMINAL:"
echo "  cd ~/sdk_deploy && source install/setup.bash"
echo "  ros2 run lightning run_slam_online --config src/lightning-lm/config/m20_sim_slam.yaml"
echo ""
echo "SAVE MAP:"
echo "  ros2 service call lightning/save_map lightning/srv/SaveMap \"{map_id: m20_sim_map}\""
echo ""
echo "PRESS Ctrl+C TO STOP"

# Wait for Ctrl+C
trap "echo 'Stopping...'; kill $IMU_PID $SIM_PID 2>/dev/null; exit" INT TERM
wait
