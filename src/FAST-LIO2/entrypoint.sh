#!/bin/bash
# If argument1 is launch: to launch all nodes, else may in exec mode

LIO_WS=/root/lio_ws
source $LIO_WS/devel/setup.bash

wait_for_roscore() {
    local timeout=$1
    local start_time=$(date +%s)

    echo "Waiting $timeout for roscore to be ready..."

    while true; do
        # 检查roscore状态
        rostopic list > /dev/null 2>&1
        if [ $? -eq 0 ]; then
            echo "roscore is ready."
            return 0
        fi

        # 检查超时时间
        if [ -n "$timeout" ]; then
            local current_time=$(date +%s)
            if (( current_time - start_time >= timeout )); then
                echo "Timeout reached while waiting for roscore."
                return -1
            fi
        fi

        sleep 1
    done
}

# Function to wait for a message on a topic with timeout
wait_for_message() {
    local topic=$1
    local TOUT=$2
    local start_time=$(date +%s)

    echo "Waiting for a message on topic $topic..."

    while true; do
        timeout $TOUT rostopic echo -n 1 $topic > /dev/null 2>&1
        if [ $? -eq 0 ]; then
            echo "Message received on topic $topic. Proceeding to next step."
            return 0
        fi

        # Check timeout condition
        local current_time=$(date +%s)
        if (( current_time - start_time >= TOUT )); then
            echo "Timeout reached while waiting for topic $topic."
            return -1
        fi
        sleep 1
    done
}

if [ "$1" == "launch" ]; then
    # Launch Livox ROS driver
    if wait_for_roscore 60; then
        echo "roscore is OK."
    else
        echo "Error while reading roscore, exiting..."
        exit -1
    fi
    roslaunch $LIO_WS/src/livox_ros_driver2/launch_ROS1/msg_MID360.launch &> /output/livox_ros_driver.log &
    echo "Checking Livox status..."

    rosrun fast_lio check_lidar_status.py --lidar /livox/lidar --only-lidar --timeout 30
    if [ $? -eq 0 ]; then
        echo "Livox is OK."
        sleep 5
    else
        echo "Error while checking Livox status... will exit."
        exit -1
    fi

    echo "Launching LIO..."
    roslaunch fast_lio mapping_mid360.launch rviz:=false root_dir:=/output driver:=false &> /output/lio.log &
    # Wait for Odometry topic
    TOPIC_ODOM="/Odometry"

    while true; do
        echo "Starting watchdog for topic $TOPIC_ODOM and auto map-saving..."
        rosrun fast_lio check_lidar_status.py --lidar /livox/lidar --odom /Odometry --timeout 10
        if [ $? -eq 0 ]; then
            echo "Livox is OK."
            sleep 10
        else
            echo "Error while checking LIO... will exit."
            exit -1
        fi
    done

elif [ "$1" == "bash" ]; then
    /bin/bash
elif [ "$1" == "save" ]; then
    rosservice call /save_lio_pcl
else
    # Execute with all arguments
    exec "$@"
fi
