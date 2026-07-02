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
                        for k_str, pose in data["location_map"].items():
                            k_int = int(k_str) 
                            if k_int in self.location_map:
                                self.location_map[k_int]['pose'] = pose
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

            loc_poses = {str(k): v['pose'] for k, v in self.location_map.items()}
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
                        self.state = 'NAVIGATING'
                        self.is_waiting = True
                        
                        self.get_logger().info(f"[Trigger] 目标分配: {self.current_target_name}。保护期起算(3.0s)...")
                        
                        def execute_delayed_nav():
                            self.get_logger().info("[System] 发起 Nav2 目标规划请求。")
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
        if status == 4: 
            self.get_logger().info('[Action] 全局导航阶段到达。')
            
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
        elif status == 6:
            self.get_logger().warn('[Action] 任务被规划器中止。')
            self.state = 'IDLE'
        elif status == 5:
            self.get_logger().warn('[Action] 任务已被取消。')
            self.state = 'IDLE'
        else:
            self.state = 'IDLE'

    # 底盘闭环控制 (驶出直行 / 泊车倒退)
    def chassis_control_loop(self):
        if self.state not in ['DOCKING', 'EXITING']:
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
                self.get_logger().info("[Control] 直行完毕，导航机彻底解锁！")
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