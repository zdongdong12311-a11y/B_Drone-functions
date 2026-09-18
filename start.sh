#!/bin/bash
source /opt/ros/noetic/setup.bash

if [ -f ~/livox_ws/devel/setup.bash ]; then
    source ~/livox_ws/devel/setup.bash --extend
fi

if [ -f ~/fast_lio2_ws/devel/setup.bash ]; then
    source ~/fast_lio2_ws/devel/setup.bash --extend
fi

if [ -f ~/trans_ws/devel/setup.bash ]; then
    source ~/trans_ws/devel/setup.bash --extend
fi

if [ -f ~/ego_ws/devel/setup.bash ]; then
    source ~/ego_ws/devel/setup.bash --extend
fi

cleanup() {
    echo ""
    echo "======================================================"
    echo "  收到退出信号 (Ctrl+C)，开始清理后台进程..."
    echo "======================================================"
    killall -INT roslaunch 2>/dev/null
    sleep 3
    killall -9 roslaunch 2>/dev/null
    killall -9 rosmaster 2>/dev/null
    killall -9 rosout 2>/dev/null
    echo "清理完成！退出。"
    exit 0
}

trap cleanup SIGINT SIGTERM

echo "[1/2] 正在启动 LiDAR 驱动、FAST-LIO2 及 MAVROS 桥接..."
roslaunch lidar_to_mavros lidar_to_mavros.launch &
PID_LIDAR=$!
sleep 8 

echo "[2/2] 正在启动 EGO-Planner 轨迹规划器..."
roslaunch ego_planner single_run_in_exp.launch &
PID_EGO=$!
sleep 5

wait $PID_LIDAR
wait $PID_EGO
