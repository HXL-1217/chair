#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
import serial
import math
import threading
import json
import os
import time          # [新增] 用于超时和延时控制
import cv2           # [新增] OpenCV 摄像头调用
from pyzbar import pyzbar # [新增] 二维码解析

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray
from tf2_ros import Buffer, TransformListener

class VoiceNavBridge(Node):
    def __init__(self):
        super().__init__('voice_nav_bridge')

        # ==========================================
        # 1. 串口硬件配置
        # ==========================================
        self.serial_port = '/dev/ttyS4'
        self.baud_rate = 115200
        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=0.5)
            self.get_logger().info(f"[Hardware] 串口通信链路已建立: {self.serial_port} (Baud: {self.baud_rate})")
        except Exception as e:
            self.get_logger().error(f"[Hardware] 串口初始化失败，请验证硬件连线与系统权限: {e}")
            return

        # ==========================================
        # 2. 目标坐标系与指令映射矩阵
        # ==========================================
        self.location_map = {
            0x32: {"name": "客厅", "pose": [-1.687, 4.518, -0.013]},
            0x33: {"name": "厨房", "pose": [4.910, 2.084, 0.031]},
            0x34: {"name": "主卧", "pose": [0.000, 0.000, 0.000]},
            0x36: {"name": "客卧", "pose": [0.000, 0.000, 0.000]},
            0x31: {"name": "厕所(仅到达)", "pose": [0.856, -2.017, -0.022]}, 
            0x35: {"name": "厕所倒车模式", "pose": [0.856, -2.017, -0.022]} 
        }

        self.toilet_dock_target = [-0.354, -1.984, -0.027] 

        # ==========================================
        # 3. 驾驶模式与速度控制指令映射
        # ==========================================
        self.mode_codes = {0x11: 0, 0x12: 1, 0x13: 2}
        self.speed_codes = {0x21: 0, 0x22: 1, 0x23: 2}
        
        self.current_mode = 0
        self.current_speed_level = 1 
        self.profile_changed = True
        self.profile_lock = threading.Lock()

        # ==========================================
        # 4. 闭环控制参数
        # ==========================================
        self.feedback_hex = 0x01 
        self.Kp_yaw = 0.6
        self.Kd_yaw = 0.3
        self.prev_yaw_error = 0.0
        self.exit_start_pose = None

        # ==========================================
        # 4b. PID 巡迹过门控制参数
        # ==========================================
        self.door_entry_pose = None
        self.door_exit_pose = None
        self.door_room_pose = None
        self.door_line_length = 0.0
        self.door_line_angle = 0.0
        self.door_start_time = None
        self.door_prev_error = 0.0

        self.Kp_cross = 1.5
        self.Kp_heading = 0.8

        # 追踪当前所在房间（用于判断是否需要先PID出门）
        self.current_room_code = None
        # 暂存待处理的目标（出发房门PID出门后再执行）
        self.pending_destination = None 

        # ==========================================
        # 5. ROS 2 接口定义
        # ==========================================
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 20)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        qos_profile = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        self.profile_pub = self.create_publisher(Int32MultiArray, '/drive_profile', qos_profile)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.chassis_timer = self.create_timer(0.05, self.chassis_control_loop)       
        self.profile_timer = self.create_timer(0.5, self.publish_profile_timer_cb)    
        
        self.state = 'IDLE' 
        self.current_target_name = "" 
        self.is_waiting = False       

        self.read_thread = threading.Thread(target=self.serial_loop)
        self.read_thread.daemon = True 
        self.read_thread.start()
        self.get_logger().info("[System] 异步语音监听守护线程已启动。")
        
        self.load_waypoints()

    # ==========================================
    # 配置文件读写逻辑
    # ==========================================
    def load_waypoints(self):
        self.config_path = '/home/orangepi/slam_ws/src/jt_chair/config/waypoints_config.json'
        
        if os.path.exists(self.config_path):
            try:
                with open(self.config_path, 'r', encoding='utf-8') as f:
                    data = json.load(f)
                    if "toilet_dock_target" in data:
                        self.toilet_dock_target = data["toilet_dock_target"]
                    if "location_map" in data:
                        for k_str, loc_data in data["location_map"].items():
                            k_int = int(k_str)
                            if k_int in self.location_map:
                                if isinstance(loc_data, dict):
                                    if 'pose' in loc_data:
                                        self.location_map[k_int]['pose'] = loc_data['pose']
                                    if 'door' in loc_data:
                                        door = loc_data['door']
                                        # 新格式 outside/inside → 自动计算方向，转为内部 entry/exit
                                        if 'outside' in door and 'inside' in door:
                                            ox, oy = door['outside'][0], door['outside'][1]
                                            ix, iy = door['inside'][0], door['inside'][1]
                                            yaw = math.atan2(iy - oy, ix - ox)
                                            door = {
                                                'entry': [ox, oy, yaw],
                                                'exit':  [ix, iy, yaw],
                                            }
                                        self.location_map[k_int]['door'] = door
                                        self.get_logger().info(
                                            f"[Config] 🚪 房间 '{self.location_map[k_int]['name']}' "
                                            f"已加载过门配置: entry={door['entry']}, "
                                            f"exit={door['exit']}"
                                        )
                                    if 'polygon' in loc_data:
                                        self.location_map[k_int]['polygon'] = loc_data['polygon']
                                        self.get_logger().info(
                                            f"[Config] 📐 房间 '{self.location_map[k_int]['name']}' "
                                            f"已加载多边形区域: {len(loc_data['polygon'])}个顶点"
                                        )
                                else:
                                    self.location_map[k_int]['pose'] = loc_data
                self.get_logger().info(f"[Config] 💾 成功从硬盘恢复历史坐标！文件路径: {self.config_path}")
            except Exception as e:
                self.get_logger().error(f"[Config] ❌ 读取配置文件失败: {e}")
        else:
            self.get_logger().info("[Config] 🆕 未检测到历史配置文件，当前使用内置默认坐标。")

    def save_waypoints(self):
        try:
            config_dir = os.path.dirname(self.config_path)
            if not os.path.exists(config_dir):
                os.makedirs(config_dir)

            loc_poses = {}
            for k, v in self.location_map.items():
                entry = {"pose": v['pose']}
                if 'door' in v:
                    # 保存为简化格式 outside/inside（仅存 x,y，方向自动计算）
                    d = v['door']
                    entry['door'] = {
                        'outside': [d['entry'][0], d['entry'][1]],
                        'inside':  [d['exit'][0],  d['exit'][1]],
                    }
                if 'polygon' in v:
                    entry['polygon'] = v['polygon']
                loc_poses[str(k)] = entry

            data = {
                "toilet_dock_target": self.toilet_dock_target,
                "location_map": loc_poses
            }
            with open(self.config_path, 'w', encoding='utf-8') as f:
                json.dump(data, f, indent=4, ensure_ascii=False)
                
            self.get_logger().info(f"[Config] 💾 最新坐标已持久化保存至硬盘: {self.config_path}")
        except Exception as e:
            self.get_logger().error(f"[Config] ❌ 坐标保存硬盘失败: {e}")

    def get_current_pose(self):
        try:
            trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            
            q = trans.transform.rotation
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            
            return [round(x, 3), round(y, 3), round(yaw, 3)]
        except Exception as e:
            self.get_logger().error(f"[Calibration] 获取当前位姿失败，TF 树可能未就绪: {e}")
            return None

    # ==========================================
    # [新增] 摄像头验证二维码专属逻辑
    # ==========================================
    def verify_toilet_qr(self):
        self.get_logger().info("📷 正在启动摄像头，开始验证厕所专属二维码 (超时时间: 6秒)...")
        cap = cv2.VideoCapture(0)  # 如果你的摄像头是 /dev/video1，请改为 1
        
        if not cap.isOpened():
            self.get_logger().error("❌ 无法打开摄像头！")
            return False

        # 稍微降低分辨率，提升香橙派的处理速度
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        success_count = 0
        target_count = 5
        timeout = 6.0
        start_time = time.time()

        try:
            while time.time() - start_time < timeout:
                ret, frame = cap.read()
                if not ret:
                    time.sleep(0.1)
                    continue
                
                # 解码画面中的二维码
                barcodes = pyzbar.decode(frame)
                for barcode in barcodes:
                    barcode_data = barcode.data.decode("utf-8")
                    
                    if barcode_data == "1":
                        success_count += 1
                        self.get_logger().info(f"🔍 [QR] 成功识别目标文本 '1' ({success_count}/{target_count})")
                        time.sleep(0.2) # 增加小延时，确保是读取了5次真实的物理扫描
                        
                if success_count >= target_count:
                    self.get_logger().info("✅ [QR] 验证成功！符合入厕标定条件。")
                    return True
                    
            self.get_logger().warn("⚠️ [QR] 验证超时！在6秒内未连续5次扫描到内容为 '1' 的二维码。")
            return False
            
        finally:
            cap.release()

    # ==========================================
    # 核心任务逻辑
    # ==========================================
    def publish_profile_timer_cb(self):
        with self.profile_lock:
            m = self.current_mode
            s = self.current_speed_level
            changed = self.profile_changed
            self.profile_changed = False

        msg = Int32MultiArray()
        msg.data = [m, s]
        self.profile_pub.publish(msg)

        if changed:
            self.get_logger().info(f"[Profile] 驾驶配置更新已发布: mode={m}, speed_level={s}")

    def serial_loop(self):
        while rclpy.ok():
            try:
                if self.ser.in_waiting > 0:
                    incoming_byte = self.ser.read(1)
                    if not incoming_byte:
                        continue
                    
                    cmd_hex = ord(incoming_byte)
                    if cmd_hex in [0xff, 0x00]:
                        continue

                    self.get_logger().info(f"[Serial] 接收到有效载荷 (Hex): {hex(cmd_hex)}")

                    # =======================================================
                    # 1. 动态标定指令解析 (0x40 - 0x45)
                    # =======================================================
                    calibration_map = {
                        0x40: "马桶", 
                        0x41: "厕所", 
                        0x42: "客厅", 
                        0x43: "厨房", 
                        0x44: "主卧", 
                        0x45: "客卧"
                    }
                    
                    if cmd_hex in calibration_map:
                        room_name = calibration_map[cmd_hex]
                        self.get_logger().info(f"[Calibration] 收到标定指令: {room_name} ({hex(cmd_hex)})...")
                        
                        # ---> [新增] 厕所坐标标定的强制二维码校验卡点 <---
                        if cmd_hex == 0x41:
                            is_verified = self.verify_toilet_qr()
                            if is_verified:
                                # 验证成功，输出文本 1
                                try:
                                    self.ser.write(b'1')
                                    self.get_logger().info("[Serial] 已向语音模块反馈文本 '1'")
                                except Exception as e:
                                    self.get_logger().error(f"[Serial] 串口发送反馈失败: {e}")
                            else:
                                # 验证失败，输出文本 0，并直接拦截此次标定
                                try:
                                    self.ser.write(b'0')
                                    self.get_logger().info("[Serial] 已向语音模块反馈文本 '0' (验证失败)")
                                except Exception as e:
                                    self.get_logger().error(f"[Serial] 串口发送反馈失败: {e}")
                                
                                self.ser.reset_input_buffer()
                                continue # 停止后续的坐标读取和保存流程
                        # --------------------------------------------------

                        current_pose = self.get_current_pose()
                        if current_pose:
                            if cmd_hex == 0x40:   
                                self.toilet_dock_target = current_pose
                            elif cmd_hex == 0x41: 
                                self.location_map[0x31]['pose'] = current_pose
                                self.location_map[0x35]['pose'] = current_pose
                            elif cmd_hex == 0x42: 
                                self.location_map[0x32]['pose'] = current_pose
                            elif cmd_hex == 0x43: 
                                self.location_map[0x33]['pose'] = current_pose
                            elif cmd_hex == 0x44: 
                                self.location_map[0x34]['pose'] = current_pose
                            elif cmd_hex == 0x45: 
                                self.location_map[0x36]['pose'] = current_pose
                                
                            self.get_logger().info(f"✅ [标定成功] {room_name} 坐标已更新: {current_pose}")
                            self.save_waypoints()
                        else:
                            self.get_logger().warn(f"❌ [标定失败] 请确认定位系统运行正常。")
                        
                        self.ser.reset_input_buffer()
                        continue

                    # =======================================================
                    # 2. 驶出解锁指令解析 (0x30)
                    # =======================================================
                    if cmd_hex == 0x30:
                        if self.state in ['LOCKED', 'IDLE']:
                            self.state = 'EXITING'
                            self.exit_start_pose = None
                            self.get_logger().info("[Trigger] 收到驶出指令(0x30)。轮椅即将直行 0.5 米并解锁导航。")
                        else:
                            self.get_logger().warn("[State] 轮椅正在运动中，忽略驶出指令。")
                        self.ser.reset_input_buffer()
                        continue

                    # =======================================================
                    # 3. 模式与速度配置指令解析
                    # =======================================================
                    if cmd_hex in self.mode_codes or cmd_hex in self.speed_codes:
                        trigger_publish = False
                        with self.profile_lock:
                            if cmd_hex in self.mode_codes:
                                new_mode = self.mode_codes[cmd_hex]
                                if self.current_mode != new_mode:
                                    self.current_mode = new_mode
                                    self.profile_changed = True
                            
                            if cmd_hex in self.speed_codes:
                                new_speed = self.speed_codes[cmd_hex]
                                if self.current_speed_level != new_speed:
                                    self.current_speed_level = new_speed
                                    self.profile_changed = True
                            
                            if self.profile_changed:
                                trigger_publish = True
                        
                        if trigger_publish:
                            self.publish_profile_timer_cb()
                        continue 

                    # =======================================================
                    # 4. 导航任务触发指令解析
                    # =======================================================
                    if cmd_hex in self.location_map:
                        if self.state == 'LOCKED':
                            self.get_logger().warn("[State] 导航处于锁定状态！必须先下发驶出指令(0x30)解锁。")
                            self.ser.reset_input_buffer()
                            continue

                        if self.state != 'IDLE':
                            self.get_logger().warn("[State] 状态机处于非空闲态，当前指令已丢弃。")
                            self.ser.reset_input_buffer()
                            continue
                            
                        if self.is_waiting:
                            self.get_logger().warn("[State] 防抖机制已拦截重复指令。")
                            self.ser.reset_input_buffer()
                            continue

                        target = self.location_map[cmd_hex]
                        self.current_target_name = target['name']

                        # --- 每次导航都检测当前所在房间（多边形判断） ---
                        detected = self._detect_current_room()
                        if detected is not None:
                            self.current_room_code = detected

                        # --- 检查是否需要先从当前房间 PID 出门 ---
                        need_source_exit = False
                        # 0x31 和 0x35 是同一个物理区域（厕所），同区域不需要出门/进门
                        same_area = (self.current_room_code in [0x31, 0x35]
                                     and cmd_hex in [0x31, 0x35])
                        if (self.current_room_code is not None
                            and self.current_room_code != cmd_hex
                            and self.current_room_code != 0x35
                            and not same_area):
                            source_room = self.location_map.get(self.current_room_code)
                            if source_room and 'door' in source_room:
                                need_source_exit = True
                                # 出门两步走: Nav2到门内侧 → PID巡迹到门外侧
                                # door.exit=inside(房内), door.entry=outside(房外)
                                ex_door = source_room['door']['exit']   # inside
                                en_door = source_room['door']['entry']  # outside
                                # 出门方向 yaw = 从inside指向outside（与进门相反）
                                exit_yaw = math.atan2(en_door[1] - ex_door[1],
                                                       en_door[0] - ex_door[0])
                                self.door_entry_pose = [ex_door[0], ex_door[1], exit_yaw]
                                self.door_exit_pose = [en_door[0], en_door[1], exit_yaw]
                                self.door_room_pose = None
                                self.state = 'NAVIGATING_TO_SOURCE_DOOR'
                                self.is_waiting = True
                                # 暂存目标信息，出门完成后再处理
                                self.pending_destination = {
                                    'cmd_hex': cmd_hex,
                                    'target': target,
                                }
                                self.get_logger().info(
                                    f"[Trigger] 先导航到 '{source_room['name']}' 门内侧"
                                    f"({self.door_entry_pose})，再PID出门。"
                                    f"目标 '{target['name']}' 待出门后执行。"
                                )

                                def execute_source_door_nav():
                                    self.get_logger().info("[System] 发起 Nav2 → 源房门内侧。")
                                    self.send_goal(self.door_entry_pose)
                                    self.is_waiting = False
                                    self.ser.reset_input_buffer()

                                timer = threading.Timer(3.0, execute_source_door_nav)
                                timer.start()
                                self.ser.reset_input_buffer()

                        if not need_source_exit:
                            # --- 无源房门，直接处理目标 ---
                            # 0x35 厕所倒车 与 0x31 厕所共享同一个物理位置和门
                            # 但如果已在同区域（厕所内），不需要再进门
                            door_target = target
                            if cmd_hex == 0x35 and 'door' not in target:
                                toilet_room = self.location_map.get(0x31)
                                if toilet_room and 'door' in toilet_room:
                                    door_target = toilet_room  # 复用 0x31 的门配置

                            has_door = ('door' in door_target and not same_area)
                            if has_door:
                                self.door_entry_pose = door_target['door']['entry']
                                self.door_exit_pose = door_target['door']['exit']
                                self.door_room_pose = target['pose']
                                self.state = 'NAVIGATING_TO_DOOR'
                                self.get_logger().info(
                                    f"[Trigger] 目标: {self.current_target_name} (含过门)。"
                                    f"门前点: {self.door_entry_pose}, 门后点: {self.door_exit_pose}"
                                )
                            else:
                                self.state = 'NAVIGATING'

                            self.is_waiting = True
                            self.get_logger().info(f"[Trigger] 目标分配: {self.current_target_name}。保护期起算(3.0s)...")

                            def execute_delayed_nav():
                                self.get_logger().info("[System] 发起 Nav2 目标规划请求。")
                                if self.state == 'NAVIGATING_TO_DOOR':
                                    self.send_goal(self.door_entry_pose)
                                else:
                                    self.send_goal(target['pose'])
                                self.is_waiting = False
                                self.ser.reset_input_buffer()

                            timer = threading.Timer(3.0, execute_delayed_nav)
                            timer.start()

                    else:
                        if cmd_hex not in self.mode_codes and cmd_hex not in self.speed_codes:
                            self.get_logger().warn(f"[Parser] 未定义指令: {hex(cmd_hex)}。")
                        
            except Exception as e:
                self.get_logger().error(f"[Serial] 串行读取异常: {e}")
                time.sleep(1.0)

    # Nav2 全局规划
    def send_goal(self, coords):
        if not self.nav_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("[Action] Nav2 服务器连接超时。")
            self.state = 'IDLE'
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = coords[0]
        goal_msg.pose.pose.position.y = coords[1]
        
        yaw = coords[2]
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.send_goal_future = self.nav_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('[Action] 目标点被规划器拒绝。')
            self.state = 'IDLE'
            return
        self.get_logger().info('[Action] 目标已接受，路径追踪开始。')
        self.goal_result_future = goal_handle.get_result_async()
        self.goal_result_future.add_done_callback(self.goal_result_callback)

    def feedback_callback(self, feedback_msg):
        pass 

    def goal_result_callback(self, future):
        status = future.result().status
        if status == 4:  # SUCCEEDED
            self.get_logger().info('[Action] 全局导航阶段到达。')

            # --- 三步流程第1步完成: 到达门前点，启动 PID 巡迹过门 ---
            if self.state == 'NAVIGATING_TO_DOOR':
                self.get_logger().info("[System] 到达门前点，启动PID巡迹过门序列。")
                self.state = 'DOOR_PASSING'
                self.door_start_time = time.time()
                self.door_prev_error = 0.0
                ex, ey = self.door_entry_pose[0], self.door_entry_pose[1]
                dx = self.door_exit_pose[0] - ex
                dy = self.door_exit_pose[1] - ey
                self.door_line_length = math.hypot(dx, dy)
                self.door_line_angle = math.atan2(dy, dx)
                self.get_logger().info(
                    f"[Door] 线段长度: {self.door_line_length:.3f}m, "
                    f"方向角: {math.degrees(self.door_line_angle):.1f}°"
                )
                return

            # --- 出门流程第1步完成: 到达源房门内侧，启动 PID 出门 ---
            if self.state == 'NAVIGATING_TO_SOURCE_DOOR':
                self.get_logger().info("[System] 到达源房门内侧，启动PID出门序列。")
                self.state = 'DOOR_EXITING'
                self.door_start_time = time.time()
                self.door_prev_error = 0.0
                ex, ey = self.door_entry_pose[0], self.door_entry_pose[1]
                dx = self.door_exit_pose[0] - ex
                dy = self.door_exit_pose[1] - ey
                self.door_line_length = math.hypot(dx, dy)
                self.door_line_angle = math.atan2(dy, dx)
                self.get_logger().info(
                    f"[Door] 出门线段长度: {self.door_line_length:.3f}m, "
                    f"方向角: {math.degrees(self.door_line_angle):.1f}°"
                )
                return

            # --- 三步流程第3步完成: 到达房间目标 ---
            if self.state == 'NAVIGATING_TO_ROOM':
                self.get_logger().info('[Action] 导航至房间目标完成。')

            # 厕所倒车模式处理 (NAVIGATING_TO_ROOM 或 NAVIGATING 都可能触发)
            if self.current_target_name == "厕所倒车模式":
                self.get_logger().info(f"[Sync] 下发同步载荷: {hex(self.feedback_hex)}")
                try:
                    self.ser.write(bytes([self.feedback_hex]))
                except Exception as e:
                    self.get_logger().error(f"[Sync] 发送失败: {e}")

                self.get_logger().info("[System] 开启泊车前保护期 (3.0s)...")

                def start_docking_sequence():
                    self.get_logger().info("[Control] 接管底盘控制权，激活闭环泊车序列。")
                    self.state = 'DOCKING'
                    self.prev_yaw_error = 0.0

                threading.Timer(3.0, start_docking_sequence).start()
            else:
                self.state = 'IDLE'
                self._update_current_room()
        elif status == 6:
            self.get_logger().warn('[Action] 任务被规划器中止。')
            self.state = 'IDLE'
        elif status == 5:
            self.get_logger().warn('[Action] 任务已被取消。')
            self.state = 'IDLE'
        else:
            self.state = 'IDLE'

    # 底盘闭环控制 (驶出直行 / 泊车倒退 / PID巡迹过门)
    def chassis_control_loop(self):
        if self.state not in ['DOCKING', 'EXITING', 'DOOR_PASSING', 'DOOR_EXITING']:
            return

        try:
            trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        except Exception:
            return

        current_x = trans.transform.translation.x
        current_y = trans.transform.translation.y
        q = trans.transform.rotation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        current_yaw = math.atan2(siny_cosp, cosy_cosp)

        # ==========================================
        # PID 巡迹过门控制 (进门 DOOR_PASSING / 出门 DOOR_EXITING)
        # ==========================================
        if self.state in ['DOOR_PASSING', 'DOOR_EXITING']:
            # 超时保护 (按线段长度动态计算, 至少30秒)
            timeout = max(30.0, self.door_line_length / 0.08 * 1.2)
            if self.door_start_time and (time.time() - self.door_start_time) > timeout:
                self.get_logger().error(f"[Door] PID巡迹超时({timeout:.0f}s)，停止并切回IDLE。")
                self.stop_robot()
                self.state = 'IDLE'
                self.door_start_time = None
                self.pending_destination = None
                return

            ex, ey = self.door_entry_pose[0], self.door_entry_pose[1]
            line_angle = self.door_line_angle

            # 横向误差 (cross-track error): 机器人到线段的垂直有向距离
            rx = current_x - ex
            ry = current_y - ey
            along_track = rx * math.cos(line_angle) + ry * math.sin(line_angle)
            cross_track = -rx * math.sin(line_angle) + ry * math.cos(line_angle)

            # 航向误差
            heading_error = math.atan2(
                math.sin(line_angle - current_yaw),
                math.cos(line_angle - current_yaw)
            )

            # 综合误差用于微分项
            combined_error = self.Kp_cross * cross_track + self.Kp_heading * heading_error
            d_error = combined_error - self.door_prev_error
            self.door_prev_error = combined_error

            # PD 控制律
            angular_z = combined_error + self.Kd_yaw * d_error
            angular_z = max(-0.5, min(0.5, angular_z))

            # 检查退出条件: 已走完90%线段 且 距出口 < 0.15m
            dist_to_exit = math.hypot(current_x - self.door_exit_pose[0],
                                       current_y - self.door_exit_pose[1])
            # 退出条件: 走过终点 或 距终点<0.2m（防止overshoot后dist反弹无法退出）
            passed_end = along_track > self.door_line_length
            near_end = dist_to_exit < 0.2
            if passed_end or near_end:
                self.get_logger().info(f"[Door] PID巡迹{'出门' if self.state == 'DOOR_EXITING' else '过门'}完成！"
                                       f"({'已过终点' if passed_end else '距终点'+str(round(dist_to_exit,2))+'m'})")
                self.stop_robot()
                self.door_start_time = None

                if self.state == 'DOOR_EXITING':
                    # 出门完成，处理暂存的目标
                    self.current_room_code = None  # 已离开原房间
                    if self.pending_destination:
                        pending = self.pending_destination
                        self.pending_destination = None
                        cmd_hex = pending['cmd_hex']
                        target = pending['target']

                        # 检查目标房间是否有门需要进
                        # 0x35 厕所倒车 与 0x31 厕所共享同一个物理位置和门
                        door_target = target
                        if cmd_hex == 0x35 and 'door' not in target:
                            toilet_room = self.location_map.get(0x31)
                            if toilet_room and 'door' in toilet_room:
                                door_target = toilet_room

                        has_door = ('door' in door_target)
                        if has_door:
                            self.door_entry_pose = door_target['door']['entry']
                            self.door_exit_pose = door_target['door']['exit']
                            self.door_room_pose = target['pose']
                            self.state = 'NAVIGATING_TO_DOOR'
                            self.get_logger().info(
                                f"[Trigger] 目标: {target['name']} (含过门)。"
                                f"门前点: {self.door_entry_pose}"
                            )
                        else:
                            self.state = 'NAVIGATING'

                        self.is_waiting = True
                        self.get_logger().info(
                            f"[Trigger] 目标分配: {target['name']}。保护期起算(3.0s)..."
                        )

                        def execute_delayed_nav():
                            self.get_logger().info("[System] 发起 Nav2 目标规划请求。")
                            if self.state == 'NAVIGATING_TO_DOOR':
                                self.send_goal(self.door_entry_pose)
                            else:
                                self.send_goal(target['pose'])
                            self.is_waiting = False
                            self.ser.reset_input_buffer()

                        timer = threading.Timer(3.0, execute_delayed_nav)
                        timer.start()
                    else:
                        self.get_logger().error("[Door] 出门后无待处理目标，切回IDLE。")
                        self.state = 'IDLE'
                else:
                    # DOOR_PASSING: 进门完成，切换至房间内导航
                    self.state = 'NAVIGATING_TO_ROOM'
                    if self.door_room_pose:
                        self.get_logger().info(
                            f"[System] 发起房间目标 Nav2 导航: {self.door_room_pose}"
                        )
                        self.send_goal(self.door_room_pose)
                    else:
                        self.get_logger().error("[Door] 房间目标位姿缺失，切回IDLE。")
                        self.state = 'IDLE'
                return

            # 前进速度 (临近出口减速)
            if dist_to_exit < 0.3:
                linear_x = 0.12 * (dist_to_exit / 0.3)
                linear_x = max(0.04, linear_x)
            else:
                linear_x = 0.12

            cmd = Twist()
            cmd.linear.x = linear_x
            cmd.angular.z = angular_z
            self.cmd_pub.publish(cmd)
            return

        # 驶出解锁控制
        if self.state == 'EXITING':
            if self.exit_start_pose is None:
                self.exit_start_pose = (current_x, current_y)
                
            dist_moved = math.hypot(current_x - self.exit_start_pose[0], current_y - self.exit_start_pose[1])
            
            if dist_moved < 0.5:
                cmd = Twist()
                cmd.linear.x = 0.15  
                cmd.angular.z = 0.0  
                self.cmd_pub.publish(cmd)
            else:
                self.stop_robot()
                self.state = 'IDLE'
                self.exit_start_pose = None
                # 驶出后默认视为在厕所区域
                self.current_room_code = 0x31
                self.get_logger().info("[Control] 直行完毕，导航机彻底解锁！当前位置: 厕所区域")
            return

        # 泊车控制
        target_x = self.toilet_dock_target[0]
        target_y = self.toilet_dock_target[1]
        target_yaw_base = self.toilet_dock_target[2]

        dx = current_x - target_x
        dy = current_y - target_y
        distance = math.hypot(dx, dy)

        e_lat = -dx * math.sin(target_yaw_base) + dy * math.cos(target_yaw_base)
        e_lon = dx * math.cos(target_yaw_base) + dy * math.sin(target_yaw_base)

        if e_lon < 0.01 or distance < 0.02:
            self.stop_robot()
            self.state = 'LOCKED' 
            self.get_logger().info(f"[Control] 泊车序列完毕！导航已被锁定。(剩余纵向: {e_lon:.3f}m)")
            return

        K_lat = 2.0 
        yaw_correction = math.atan(K_lat * e_lat)
        yaw_correction = max(-0.6, min(0.6, yaw_correction))
        target_yaw_dynamic = target_yaw_base + yaw_correction
        yaw_error = math.atan2(math.sin(target_yaw_dynamic - current_yaw), math.cos(target_yaw_dynamic - current_yaw))
        
        if abs(yaw_error) < 0.03:
            yaw_error = 0.0

        d_error = yaw_error - self.prev_yaw_error
        raw_angular_z = (self.Kp_yaw * yaw_error) + (self.Kd_yaw * d_error)
        self.prev_yaw_error = yaw_error

        angular_z = raw_angular_z
        linear_x = -0.15 
        
        if distance < 0.40:
            linear_x = -0.04 - (distance / 0.40) * 0.11
            angular_z = raw_angular_z * (distance / 0.40)
            
        angular_z = max(-0.4, min(0.4, angular_z))

        cmd = Twist()
        cmd.linear.x = linear_x
        cmd.angular.z = angular_z 
        self.cmd_pub.publish(cmd)

    def stop_robot(self):
        self.cmd_pub.publish(Twist())

    def _update_current_room(self):
        """根据 last navigation target name 更新当前所在房间"""
        for code, info in self.location_map.items():
            if info['name'] == self.current_target_name:
                self.current_room_code = code
                self.get_logger().info(f"[State] 当前位置已记录: {info['name']} (0x{code:02X})")
                return
        self.current_room_code = None

    @staticmethod
    def _point_in_polygon(px, py, polygon):
        """射线法判断点 (px, py) 是否在多边形内。
        polygon: [[x1,y1], [x2,y2], ...] 顶点列表（自动闭合）"""
        n = len(polygon)
        if n < 3:
            return False
        inside = False
        j = n - 1
        for i in range(n):
            xi, yi = polygon[i][0], polygon[i][1]
            xj, yj = polygon[j][0], polygon[j][1]
            if ((yi > py) != (yj > py)) and (px < (xj - xi) * (py - yi) / (yj - yi) + xi):
                inside = not inside
            j = i
        return inside

    def _detect_current_room(self):
        """根据机器人当前坐标 + 多边形区域判断在哪个房间"""
        pose = self.get_current_pose()
        if pose is None:
            self.get_logger().warn("[Detect] 无法获取当前位姿，跳过房间检测。")
            return None

        rx, ry = pose[0], pose[1]

        for code, info in self.location_map.items():
            if code == 0x35:
                continue
            if 'polygon' not in info:
                continue
            if self._point_in_polygon(rx, ry, info['polygon']):
                self.get_logger().info(
                    f"[Detect] 📐 多边形检测: 机器人在 '{info['name']}' "
                    f"(0x{code:02X}) 区域内"
                )
                return code

        # 没有落在任何已标定的多边形内 → 视为公共区域
        self.get_logger().info("[Detect] 机器人不在任何已标定房间多边形内，视为公共区域。")
        return None

def main(args=None):
    rclpy.init(args=args)
    node = VoiceNavBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()