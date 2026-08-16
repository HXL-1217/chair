#!/bin/bash

# ==========================================
# 离线建图启动脚本 (从 rosbag 回放重建地图)
# 用法: ./start_offline_mapping.sh [bag目录] [回放倍速] [地图保存路径前缀]
#   例: ./start_offline_mapping.sh ~/slam_ws/rosbags/mapping_20260816
# ==========================================

BAG_PATH="${1:-$HOME/slam_ws/rosbags/mapping_20260816}"
PLAY_RATE="${2:-1.0}"
SAVE_MAP_PREFIX="${3:-$HOME/slam_ws/maps/offline_map}"

# 1. 定义清理函数（捕获到 Ctrl+C 时触发）
cleanup() {
    echo ""
    echo "[offline-mapping] 正在停止离线建图..."
    kill "$PLAY_PID" "$EKF_PID" "$MERGE_PID" "$SLAM_PID" 2>/dev/null
    pkill -f static_transform_publisher 2>/dev/null
    pkill -9 -f "slam_toolbox" 2>/dev/null
    pkill -9 -f "ekf_node" 2>/dev/null
    pkill -9 -f "dual_laser_merger" 2>/dev/null
    pkill -9 -f "ros2 bag play" 2>/dev/null
    echo "[offline-mapping] 清理完成！"
    exit 0
}

# 2. 注册信号捕获器
trap cleanup SIGINT SIGTERM

# 3. 环境初始化
source /opt/ros/humble/setup.bash
source ~/slam_ws/install/setup.bash

mkdir -p ~/slam_ws/maps

echo "[offline-mapping] 回放 rosbag: $BAG_PATH (倍速: $PLAY_RATE)"

# 4. 先启动处理节点（静态TF + EKF + 双雷达融合 + 离线SLAM）
#    静态 TF 数值与 double_nav2.launch.py 一致
ros2 run tf2_ros static_transform_publisher 0.29 -0.255 0.2 0 0 0 base_link laser_1 &
ros2 run tf2_ros static_transform_publisher -0.29 0.255 0.2 3.124 0 0 base_link laser_2 &
ros2 run tf2_ros static_transform_publisher 0.185 0 0.2 0 0 0 base_link imu_link &

# EKF：由 /odom + /imu 恢复 odom->base_link TF
ros2 run robot_localization ekf_node --ros-args \
    --params-file ~/slam_ws/src/jt_chair/config/ekf.yaml \
    -r /imu/data:=/imu \
    -p use_sim_time:=true &
EKF_PID=$!

# 双雷达融合：恢复 /scan_merged
ros2 run dual_laser_merger dual_laser_merger_node --ros-args \
    -p laser_1_topic:=/scan_1 -p laser_2_topic:=/scan_2 \
    -p merged_scan_topic:=/scan_merged -p target_frame:=base_link \
    -p publish_rate:=20 -p angle_increment:=0.00698 -p scan_time:=0.1 \
    -p range_min:=0.20 -p range_max:=25.0 -p use_inf:=False \
    -p use_sim_time:=true &
MERGE_PID=$!

# slam_toolbox 离线建图
ros2 run slam_toolbox sync_slam_toolbox_node --ros-args \
    --params-file ~/slam_ws/src/jt_chair/config/mapper_params_offline.yaml \
    -p use_sim_time:=true &
SLAM_PID=$!

# 等节点全部起来再回放，避免丢开头几帧
sleep 3

# 5. 回放 rosbag（必须从开头播，EKF imu0_relative 依赖第一帧 Yaw 基准）
ros2 bag play --clock -r "$PLAY_RATE" "$BAG_PATH" &
PLAY_PID=$!

echo "[offline-mapping] 离线建图运行中，按 Ctrl+C 中断。"
echo "[offline-mapping] 回放结束后将自动保存地图。"

# 6. 阻塞等待回放结束
wait "$PLAY_PID"

# 7. rosbag 回放结束，等 2 秒让 slam_toolbox 完成回环收尾，然后保存地图
sleep 2
echo "[offline-mapping] 回放结束，保存地图到 $SAVE_MAP_PREFIX ..."
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: '$SAVE_MAP_PREFIX'}"

sleep 1
cleanup
