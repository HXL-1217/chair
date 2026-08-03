#!/bin/bash
# ==========================================
# jt_chair_web 启动脚本
# ==========================================

cleanup() {
    echo ""
    echo "[web] 接收到退出信号，正在关闭 Web 服务..."

    # 杀掉占用5000端口的进程及其子进程
    PORT_PID=$(fuser 5000/tcp 2>/dev/null | tr -d ' ')
    if [ -n "$PORT_PID" ]; then
        echo "[web] 正在终止进程 (PID: $PORT_PID)..."
        kill -INT $PORT_PID 2>/dev/null
        sleep 1
        # 如果还没死，强制杀
        if fuser 5000/tcp &>/dev/null; then
            echo "[web] 强制终止..."
            fuser -k 5000/tcp 2>/dev/null
        fi
    fi

    # 清理残留的 python3 app.py
    pkill -f "python3 app.py" 2>/dev/null

    echo "[web] Web 服务已关闭"
    exit 0
}

trap cleanup SIGINT SIGTERM

# 清理可能残留的旧进程（占用5000端口）
OLD_PID=$(fuser 5000/tcp 2>/dev/null | tr -d ' ')
if [ -n "$OLD_PID" ]; then
    echo "[web] 检测到端口5000被占用 (PID: $OLD_PID)，正在清理旧进程..."
    fuser -k 5000/tcp 2>/dev/null
    sleep 1
fi

source /opt/ros/humble/setup.bash
source ~/slam_ws/install/setup.bash

echo "============================================"
echo "  智能轮椅 Web 控制台"
echo "  启动中..."
echo "============================================"

cd /home/orangepi/slam_ws/src/jt_chair_web
python3 app.py &
WEB_PID=$!

echo "[web] Web 服务 PID: $WEB_PID"
echo "[web] 请在浏览器中打开: http://$(hostname -I | awk '{print $1}'):5000"
echo "[web] 按 Ctrl+C 退出"

# 不直接用 wait（会卡死），改用轮询
while kill -0 $WEB_PID 2>/dev/null; do
    sleep 1
done
echo "[web] Web 服务已退出"
