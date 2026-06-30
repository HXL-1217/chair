#!/bin/bash

# ==========================================
# RK3588S 智能轮椅建图优化启动脚本 (优雅退出版)
# ==========================================

# 1. 定义清理函数（捕获到 Ctrl+C 时触发）
cleanup() {
    echo ""
    
    echo "[dsw_chair-5] 接收到退出信号 ，正在安全停止建图..."

    if [ -n "$LAUNCH_PID" ]; then
        echo "[dsw_chair-5] 正在关闭所有相关节点 (PID: $LAUNCH_PID)..."
        
        # 尝试杀掉整个进程组
        pkill -P "$LAUNCH_PID"
        kill -INT "$LAUNCH_PID" 2>/dev/null
        
        # 给系统 2 秒钟释放端口
        sleep 2
        
        # 终极保险：强杀常见的建图残留进程
        pkill -9 -f "slam_toolbox" 2>/dev/null
        pkill -9 -f "robot_state_publisher" 2>/dev/null
    fi

    echo "[dsw_chair-5] 清理完成！"
    exit 0
}

# 2. 注册信号捕获器
trap cleanup SIGINT SIGTERM

# ==========================================
# 主流程开始
# ==========================================

# 3. 环境初始化
source /opt/ros/humble/setup.bash
source ~/slam_ws/install/setup.bash

# 4. 性能调优环境变量
export OMP_NUM_THREADS=4 
export OPENCV_VIDEOIO_PRIORITY_MSMF=0


echo "[dsw_chair-5] 正在启动双雷达建图系统..."

# 5. 启动 Launch 文件
# 注意你给的脚本里叫 double_mapping.launch.py，我保留了你的命名
ros2 launch jt_chair double_mapping.launch.py &

# 获取 Launch 进程 PID
LAUNCH_PID=$!

echo "[dsw_chair-5] 等待节点完全加载 (5s)..."
sleep 5

# 6. 动态性能优化与绑核
SLAM_PID=$(pgrep -n -f "slam_toolbox")

if [ -n "$SLAM_PID" ]; then
    echo "[dsw_chair-5] 发现 SLAM 进程 (PID: $SLAM_PID)，正在分配至 A76 大核 (4-7)..."
    
    # 调整核心与优先级 (依赖免密 sudo)
    sudo taskset -pc 4-7 $SLAM_PID > /dev/null
    sudo renice -n -10 -p $SLAM_PID > /dev/null
    
    echo "    [dsw_chair-5] SLAM 性能优化指令已下达"
else
    echo "    [dsw_chair-5] 未找到 SLAM 进程，它可能启动失败或名字不匹配"
fi


echo "    [dsw_chair-5] 建图系统运行中..."
echo "    [dsw_chair-5] 按 Ctrl+C 可一键停止并清理所有节点。"


# 7. 阻塞主进程，等待子进程或信号
wait