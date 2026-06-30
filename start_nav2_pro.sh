#!/bin/bash

# ==========================================
# 优雅退出版 - 导航启动脚本
# ==========================================

# 1. 定义清理函数（当收到退出信号时执行）
cleanup() {
    echo "" 
    echo "[nav2] 接收到退出信号 (Ctrl+C)，开始优雅清理..."

    # 杀掉后台运行的 Launch 进程及其所有子进程
    # 使用 -负号 PID 可以杀掉整个进程组 (前提是启动时开启了单独的进程组)
    if [ -n "$LAUNCH_PID" ]; then
        echo " [nav2] 正在发送终止信号..."
        # 尝试杀掉通过这个 launch 启动的整个进程树
        pkill -P "$LAUNCH_PID"
        kill -INT "$LAUNCH_PID" 2>/dev/null
        
        sleep 2
        
        # 终极保险：强行清理常见的 ROS2 僵尸进程
        pkill -9 -f "controller_server" 2>/dev/null
        pkill -9 -f "planner_server" 2>/dev/null
        pkill -9 -f "bt_navigator" 2>/dev/null
    fi

    echo " [nav2] 清理完成！再见！"
    exit 0
}

# 2. 捕获中断信号 (Ctrl+C) 和终止信号
trap cleanup SIGINT SIGTERM

# ==========================================
# 主流程开始
# ==========================================

source /opt/ros/humble/setup.bash
source ~/slam_ws/install/setup.bash
export OMP_NUM_THREADS=4

echo " 正在启动 Nav2 MPPI 导航系统..."

# 3. 启动 Launch（注意：这里不用特殊技巧，正常的 & 即可，因为我们有了 Trap）
ros2 launch jt_chair double_nav2.launch.py &

# 获取刚才后台运行的 Launch 进程的 PID
LAUNCH_PID=$!

echo " [nav2] 等待节点初始化 (10s)..."
# 注意：在 wait 或 sleep 期间按 Ctrl+C，Trap 依然能捕获到
sleep 10

# ---------- 性能优化部分 ----------
target_nodes=("controller_server" "planner_server")

for node in "${target_nodes[@]}"; do
    NODE_PID=$(pgrep -n -f "$node")
    if [ -n "$NODE_PID" ]; then
        echo " [nav2] 正在分配 $node (PID: $NODE_PID) 至大核..."
        sudo taskset -pc 4-7 $NODE_PID > /dev/null
        sudo renice -n -15 -p $NODE_PID > /dev/null
        echo "    [nav2] $node 优化完成"
    fi
done
# ----------------------------------


echo " [nav2] 导航准备就绪！按 Ctrl+C 退出。"


# 4. 阻塞脚本，等待所有的后台任务
# wait 不带参数会等待所有的后台进程结束。
# 当按 Ctrl+C 时，wait 会被打断，然后执行 trap 定义的 cleanup 函数
wait