#!/usr/bin/env python3
"""
jt_chair_web - 智能轮椅 Web 用户界面
Flask + SocketIO + ROS2 桥接
"""

import os
import sys
import json
import time
import math
import base64
import signal
import subprocess
import threading
from io import BytesIO
from pathlib import Path

import numpy as np
from PIL import Image

# --- Flask ---
from flask import Flask, render_template, request, jsonify
from flask_socketio import SocketIO, emit

# --- ROS2 ---
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from rclpy.executors import MultiThreadedExecutor

from nav_msgs.msg import OccupancyGrid, Path as NavPath, Odometry
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, PoseStamped, Point, Quaternion
from std_msgs.msg import Int32, Int32MultiArray
from nav2_msgs.action import NavigateToPose
from tf2_ros import Buffer, TransformListener, TransformException

# ==========================================
# 路径配置
# ==========================================
WORKSPACE_DIR = '/home/orangepi/slam_ws'
CONFIG_PATH = os.path.join(WORKSPACE_DIR, 'src/jt_chair/config/waypoints_config.json')
NAV2_SCRIPT = os.path.join(WORKSPACE_DIR, 'start_nav2_pro.sh')
JOY_SCRIPT = os.path.join(WORKSPACE_DIR, 'handle/joy.py')

# 房间编码映射（与 voice_nav_bridge.py 对齐）
ROOM_MAP = {
    "living_room":  {"code": 50, "name": "客厅", "hex": 0x32},
    "kitchen":      {"code": 51, "name": "厨房", "hex": 0x33},
    "master_bedroom": {"code": 52, "name": "主卧", "hex": 0x34},
    "guest_bedroom":  {"code": 54, "name": "客卧", "hex": 0x36},
    "toilet":       {"code": 49, "name": "厕所", "hex": 0x31},
    "toilet_dock":  {"code": None, "name": "马桶", "hex": 0x35},  # 保存到 toilet_dock_target
}

# ==========================================
# Flask 应用初始化
# ==========================================
app = Flask(__name__)
app.config['SECRET_KEY'] = 'jt_chair_web_secret'
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='threading')

# ==========================================
# 全局共享状态（线程安全）
# ==========================================
class SharedState:
    def __init__(self):
        self.lock = threading.Lock()
        self.map_image_base64 = None      # 地图 PNG base64
        self.map_info = None              # {width, height, resolution, origin_x, origin_y, origin_yaw}
        self.robot_pose = None            # {x, y, yaw}
        self.global_path = []             # [{x, y}, ...]
        self.nav2_running = False
        self.cmdvel_running = False
        self.mode = 0                     # 0=禁止横移, 1=禁止旋转, 2=仅旋转
        self.speed_level = 1              # 0=低速, 1=中速, 2=高速
        self.current_map_topic = None     # 最新 map 名称（用于判断更新）

state = SharedState()

# ==========================================
# ROS2 节点（在后台线程运行）
# ==========================================
class ChairWebRosNode(Node):
    def __init__(self):
        super().__init__('chair_web_bridge')

        # QoS - 使用 TRANSIENT_LOCAL 以获取持久化地图
        map_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )

        # --- 订阅 ---
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, map_qos)
        self.path_sub = self.create_subscription(
            NavPath, '/plan', self.path_callback, 10)

        # --- 发布 ---
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)
        self.profile_pub = self.create_publisher(
            Int32MultiArray, '/drive_profile', QoSProfile(
                depth=1,
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))

        # 座椅电机发布
        self.motor_pubs = {}
        for i in range(1, 6):
            self.motor_pubs[i] = self.create_publisher(Int32, f'/motor_{i}/cmd', 10)

        # --- Action 客户端 ---
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # --- TF ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- 定时器 ---
        self.tf_timer = self.create_timer(0.1, self.tf_timer_cb)  # 10Hz
        self.broadcast_timer = self.create_timer(0.2, self.broadcast_state_cb)  # 5Hz

        self.get_logger().info('Chair Web Bridge 节点已启动')

    def map_callback(self, msg: OccupancyGrid):
        """将 OccupancyGrid 转为 PNG base64"""
        try:
            width = msg.info.width
            height = msg.info.height
            resolution = msg.info.resolution

            # 原始数据
            raw = np.array(msg.data, dtype=np.int8).reshape(height, width)

            # 创建 RGBA 图像
            img = np.zeros((height, width, 4), dtype=np.uint8)

            # -1 (unknown) → 浅灰色 (204,204,204,255)
            # 0 (free) → 白色 (255,255,255,255)
            # 1-100 (occupied) → 黑色渐变
            unknown_mask = raw == -1
            free_mask = raw == 0
            occupied_mask = raw > 0

            img[unknown_mask] = [204, 204, 204, 255]
            img[free_mask] = [255, 255, 255, 255]
            # 障碍物：从深灰到黑色
            occ_vals = raw[occupied_mask].astype(np.float32)
            gray = (255 - occ_vals / 100.0 * 255).astype(np.uint8)
            img[occupied_mask, 0] = gray
            img[occupied_mask, 1] = gray
            img[occupied_mask, 2] = gray
            img[occupied_mask, 3] = 255

            # 不翻转 Y 轴：Canvas 的 Y 轴向下增长，
            # 地图坐标系 Y 轴向上增长，两者的自然映射已经正确
            # OccupancyGrid row 0 = 地图底部，Image row 0 = Canvas 顶部
            # drawImage 从 top-left 向下绘制，正好与地图底部→顶部对应

            pil_img = Image.fromarray(img, 'RGBA')

            # 压缩为 PNG
            buf = BytesIO()
            pil_img.save(buf, format='PNG', optimize=True)
            buf.seek(0)
            b64 = base64.b64encode(buf.read()).decode('utf-8')

            map_info = {
                'width': width,
                'height': height,
                'resolution': resolution,
                'origin_x': msg.info.origin.position.x,
                'origin_y': msg.info.origin.position.y,
                'origin_yaw': 2.0 * math.atan2(
                    msg.info.origin.orientation.z,
                    msg.info.origin.orientation.w) if msg.info.origin.orientation.w != 0 else 0.0
            }

            with state.lock:
                state.map_image_base64 = b64
                state.map_info = map_info

        except Exception as e:
            self.get_logger().error(f'地图转换失败: {e}')

    def path_callback(self, msg: NavPath):
        """解析全局规划路径"""
        path_points = []
        for p in msg.poses:
            path_points.append({
                'x': round(p.pose.position.x, 3),
                'y': round(p.pose.position.y, 3)
            })
        with state.lock:
            state.global_path = path_points

    def tf_timer_cb(self):
        """通过 TF 获取 map→base_link 变换"""
        try:
            trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            q = trans.transform.rotation
            yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            )
            with state.lock:
                state.robot_pose = {'x': round(x, 3), 'y': round(y, 3), 'yaw': round(yaw, 3)}
        except TransformException:
            pass  # TF 还未就绪
        except Exception as e:
            self.get_logger().debug(f'TF 查询异常: {e}')

    def broadcast_state_cb(self):
        """定期通过 WebSocket 广播状态"""
        with state.lock:
            map_b64 = state.map_image_base64
            map_info = state.map_info
            robot_pose = state.robot_pose
            global_path = list(state.global_path)
            nav2_running = state.nav2_running
            cmdvel_running = state.cmdvel_running
            mode = state.mode
            speed_level = state.speed_level

        if map_info is not None or robot_pose is not None:
            socketio.emit('state_update', {
                'map_image': map_b64,
                'map_info': map_info,
                'robot_pose': robot_pose,
                'global_path': global_path,
                'nav2_running': nav2_running,
                'cmdvel_running': cmdvel_running,
                'mode': mode,
                'speed_level': speed_level,
            })

    def set_initial_pose(self, x, y, yaw):
        """发布初始位姿到 /initialpose"""
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0
        # yaw → quaternion
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        # 协方差
        msg.pose.covariance = [
            0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0685
        ]
        self.initial_pose_pub.publish(msg)
        self.get_logger().info(f'初始位姿已设置: ({x:.2f}, {y:.2f}, yaw={yaw:.2f})')

    def send_nav_goal(self, x, y, yaw):
        """通过 Nav2 Action 发送导航目标"""
        if not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error('Nav2 Action 服务器不可用')
            return False

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.nav_client.send_goal_async(goal_msg)
        self.get_logger().info(f'导航目标已发送: ({x:.2f}, {y:.2f}, yaw={yaw:.2f})')
        return True

    def set_profile(self, mode, speed_level):
        """发布 /drive_profile"""
        msg = Int32MultiArray()
        msg.data = [mode, speed_level]
        self.profile_pub.publish(msg)
        with state.lock:
            state.mode = mode
            state.speed_level = speed_level
        self.get_logger().info(f'驾驶配置: mode={mode}, speed={speed_level}')

    def control_motor(self, motor_id, percent):
        """发布座椅电机指令"""
        if motor_id in self.motor_pubs:
            msg = Int32()
            msg.data = int(percent)
            self.motor_pubs[motor_id].publish(msg)

    def cancel_nav(self):
        """取消当前导航（发布 0 速度即可中断）"""
        # 注意：更好的是调用 cancel action, 但这里先简单停止
        self.get_logger().info('导航已取消')


# 全局 ROS 节点引用
ros_node = None

# ==========================================
# 进程管理
# ==========================================
nav2_process = None
joy_process = None

def start_nav2():
    global nav2_process
    if nav2_process is not None and nav2_process.poll() is None:
        return False, "导航程序已在运行中"
    try:
        nav2_process = subprocess.Popen(
            ['bash', NAV2_SCRIPT],
            cwd=WORKSPACE_DIR,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            preexec_fn=os.setsid  # 创建独立进程组
        )
        with state.lock:
            state.nav2_running = True
        return True, "导航程序已启动"
    except Exception as e:
        return False, f"启动失败: {e}"

def stop_nav2():
    global nav2_process
    if nav2_process is None or nav2_process.poll() is not None:
        with state.lock:
            state.nav2_running = False
        return False, "导航程序未在运行"

    try:
        # 发送 SIGINT 到整个进程组（模拟 Ctrl+C）
        os.killpg(os.getpgid(nav2_process.pid), signal.SIGINT)
        try:
            nav2_process.wait(timeout=8)
        except subprocess.TimeoutExpired:
            # 强制杀死
            os.killpg(os.getpgid(nav2_process.pid), signal.SIGKILL)
            nav2_process.wait()
    except ProcessLookupError:
        pass
    except Exception as e:
        pass

    nav2_process = None
    with state.lock:
        state.nav2_running = False
    return True, "导航程序已停止"

def start_joy():
    """启动 joy.py 手柄控制节点（joy_cmdvel_with_profile）"""
    global joy_process
    if joy_process is not None and joy_process.poll() is None:
        return False, "手柄控制已在运行中"
    try:
        env = os.environ.copy()
        env['SDL_VIDEODRIVER'] = 'dummy'
        # 先 source ROS2 环境，然后运行 joy.py
        cmd = f"source /opt/ros/humble/setup.bash && source {WORKSPACE_DIR}/install/setup.bash && python3 {JOY_SCRIPT}"
        joy_process = subprocess.Popen(
            ['bash', '-c', cmd],
            cwd=os.path.dirname(JOY_SCRIPT),
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            preexec_fn=os.setsid,
            env=env
        )
        with state.lock:
            state.cmdvel_running = True
        return True, "手柄控制已启动 (joy_cmdvel_with_profile)"
    except Exception as e:
        return False, f"启动失败: {e}"

def stop_joy():
    """停止 joy.py 手柄控制节点"""
    global joy_process
    if joy_process is None or joy_process.poll() is not None:
        with state.lock:
            state.cmdvel_running = False
        return False, "手柄控制未在运行"

    try:
        os.killpg(os.getpgid(joy_process.pid), signal.SIGINT)
        try:
            joy_process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(joy_process.pid), signal.SIGKILL)
            joy_process.wait()
    except ProcessLookupError:
        pass
    except Exception:
        pass

    joy_process = None
    with state.lock:
        state.cmdvel_running = False
    return True, "手柄控制已停止"

# ==========================================
# waypoints 读写
# ==========================================
def load_waypoints():
    """读取 waypoints_config.json"""
    if os.path.exists(CONFIG_PATH):
        with open(CONFIG_PATH, 'r', encoding='utf-8') as f:
            return json.load(f)
    return {"toilet_dock_target": [0.0, 0.0, 0.0], "location_map": {}}

def save_waypoints(data):
    """保存 waypoints_config.json"""
    config_dir = os.path.dirname(CONFIG_PATH)
    os.makedirs(config_dir, exist_ok=True)
    with open(CONFIG_PATH, 'w', encoding='utf-8') as f:
        json.dump(data, f, indent=4, ensure_ascii=False)
    return True

def record_room_pose(room_key):
    """记录当前轮椅位姿到指定房间"""
    with state.lock:
        pose = state.robot_pose

    if pose is None:
        return False, "当前位姿不可用，请确保定位系统运行正常"

    data = load_waypoints()

    if room_key == "toilet_dock":
        # 马桶 → toilet_dock_target
        data["toilet_dock_target"] = [pose['x'], pose['y'], pose['yaw']]
        save_waypoints(data)
        return True, f"马桶停靠点已记录: ({pose['x']:.3f}, {pose['y']:.3f})"

    room = ROOM_MAP.get(room_key)
    if not room or room['code'] is None:
        return False, f"未知房间: {room_key}"

    code_str = str(room['code'])
    new_pose = [pose['x'], pose['y'], pose['yaw']]

    if "location_map" not in data:
        data["location_map"] = {}

    if code_str in data["location_map"]:
        if isinstance(data["location_map"][code_str], dict):
            data["location_map"][code_str]["pose"] = new_pose
        else:
            data["location_map"][code_str] = {"pose": new_pose}
    else:
        data["location_map"][code_str] = {"pose": new_pose}

    # 厕所也更新 53（厕所倒车模式）
    if room_key == "toilet":
        if "53" not in data["location_map"]:
            data["location_map"]["53"] = {}
        if isinstance(data["location_map"]["53"], dict):
            data["location_map"]["53"]["pose"] = new_pose
        else:
            data["location_map"]["53"] = {"pose": new_pose}

    save_waypoints(data)
    return True, f"{room['name']}坐标已记录: ({pose['x']:.3f}, {pose['y']:.3f}, yaw={pose['yaw']:.3f})"

# ==========================================
# Flask 路由
# ==========================================
@app.route('/')
def index():
    return render_template('index.html')

@app.route('/api/status')
def api_status():
    with state.lock:
        return jsonify({
            'nav2_running': state.nav2_running,
            'cmdvel_running': state.cmdvel_running,
            'mode': state.mode,
            'speed_level': state.speed_level,
            'robot_pose': state.robot_pose,
        })

# --- 导航启停 ---
@app.route('/api/nav2/start', methods=['POST'])
def api_nav2_start():
    ok, msg = start_nav2()
    return jsonify({'success': ok, 'message': msg})

@app.route('/api/nav2/stop', methods=['POST'])
def api_nav2_stop():
    ok, msg = stop_nav2()
    return jsonify({'success': ok, 'message': msg})

# --- 手柄控制启停 (joy.py) ---
@app.route('/api/cmdvel/start', methods=['POST'])
def api_cmdvel_start():
    ok, msg = start_joy()
    return jsonify({'success': ok, 'message': msg})

@app.route('/api/cmdvel/stop', methods=['POST'])
def api_cmdvel_stop():
    ok, msg = stop_joy()
    return jsonify({'success': ok, 'message': msg})

# --- 模式/速度切换 ---
@app.route('/api/profile', methods=['POST'])
def api_profile():
    data = request.get_json()
    mode = data.get('mode', 0)
    speed_level = data.get('speed_level', 1)
    if ros_node:
        ros_node.set_profile(mode, speed_level)
    return jsonify({'success': True, 'mode': mode, 'speed_level': speed_level})

# --- 初始位姿设定 ---
@app.route('/api/initialpose', methods=['POST'])
def api_initialpose():
    data = request.get_json()
    x = data.get('x', 0.0)
    y = data.get('y', 0.0)
    yaw = data.get('yaw', 0.0)
    if ros_node:
        ros_node.set_initial_pose(x, y, yaw)
    return jsonify({'success': True})

# --- 导航目标设定 ---
@app.route('/api/navgoal', methods=['POST'])
def api_navgoal():
    data = request.get_json()
    x = data.get('x', 0.0)
    y = data.get('y', 0.0)
    yaw = data.get('yaw', 0.0)
    if ros_node:
        ok = ros_node.send_nav_goal(x, y, yaw)
        return jsonify({'success': ok})
    return jsonify({'success': False, 'message': 'ROS 节点未就绪'})

# --- 取消导航 ---
@app.route('/api/navgoal/cancel', methods=['POST'])
def api_navgoal_cancel():
    if ros_node:
        ros_node.cancel_nav()
    return jsonify({'success': True})

# --- 记录房间 ---
@app.route('/api/record_room', methods=['POST'])
def api_record_room():
    data = request.get_json()
    room_key = data.get('room', '')
    ok, msg = record_room_pose(room_key)
    return jsonify({'success': ok, 'message': msg})

# --- 获取当前所有航点 ---
@app.route('/api/waypoints', methods=['GET'])
def api_get_waypoints():
    data = load_waypoints()
    return jsonify(data)

# --- 座椅电机控制 ---
@app.route('/api/seat', methods=['POST'])
def api_seat():
    data = request.get_json()
    motor_id = data.get('motor_id', 1)   # 默认电机1
    percent = data.get('percent', 0)      # -100 到 100
    if ros_node:
        ros_node.control_motor(motor_id, percent)
    return jsonify({'success': True})

# --- SocketIO 事件 ---
@socketio.on('connect')
def handle_connect():
    print('WebSocket 客户端已连接')
    # 推送当前状态
    with state.lock:
        emit('state_update', {
            'map_image': state.map_image_base64,
            'map_info': state.map_info,
            'robot_pose': state.robot_pose,
            'global_path': list(state.global_path),
            'nav2_running': state.nav2_running,
            'cmdvel_running': state.cmdvel_running,
            'mode': state.mode,
            'speed_level': state.speed_level,
        })

@socketio.on('disconnect')
def handle_disconnect():
    print('WebSocket 客户端已断开')

# ==========================================
# ROS2 后台线程
# ==========================================
def ros_spin_thread():
    """在后台线程中运行 ROS2 executor"""
    global ros_node
    rclpy.init(args=sys.argv)
    ros_node = ChairWebRosNode()
    executor = MultiThreadedExecutor()
    executor.add_node(ros_node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'[ROS2] Executor 异常: {e}')
    finally:
        try:
            executor.shutdown()
        except Exception:
            pass
        try:
            if ros_node is not None:
                ros_node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass

# ==========================================
# 启动入口
# ==========================================
def main():
    # 启动 ROS2 后台线程
    ros_thread = threading.Thread(target=ros_spin_thread, daemon=True)
    ros_thread.start()

    # 等待 ROS2 就绪
    time.sleep(2)

    # 启动 Flask + SocketIO
    print("=" * 50)
    print("  jt_chair_web 智能轮椅 Web 界面")
    print("  请在浏览器中打开: http://<OrangePi_IP>:5000")
    print("=" * 50)
    socketio.run(app, host='0.0.0.0', port=5000, debug=False, allow_unsafe_werkzeug=True)

if __name__ == '__main__':
    main()
