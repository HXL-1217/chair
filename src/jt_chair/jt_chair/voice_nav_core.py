# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# import serial
# import math
# import threading
# import time
# import cv2
# from pyzbar import pyzbar

# from geometry_msgs.msg import Twist, PoseStamped
# from std_msgs.msg import Int32MultiArray
# from tf2_ros import Buffer, TransformListener

# # 导入解耦的模块
# from jt_chair.waypoint_manager import WaypointManager
# from jt_chair.nav2_commander import Nav2Commander

# class VoiceNavBridge(Node):
#     def __init__(self):
#         super().__init__('voice_nav_bridge')

#         # 1. 实例化数据层和规划层
#         config_path = '/home/orangepi/slam_ws/src/jt_chair/config/waypoints_config.json'
#         self.db = WaypointManager(self.get_logger(), config_path)
#         self.nav2 = Nav2Commander(self, self.on_nav2_success, self.on_nav2_fail)

#         # 2. 硬件串口配置
#         self.serial_port = '/dev/ttyS4'
#         self.baud_rate = 115200
#         try:
#             self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=0.5)
#             self.get_logger().info(f"[Hardware] 串口通信链路已建立: {self.serial_port} (Baud: {self.baud_rate})")
#         except Exception as e:
#             self.get_logger().error(f"[Hardware] 串口初始化失败，请验证硬件连线与系统权限: {e}")
#             return

#         # 3. 驾驶模式与速度控制指令映射 (恢复)
#         self.mode_codes = {0x11: 0, 0x12: 1, 0x13: 2}
#         self.speed_codes = {0x21: 0, 0x22: 1, 0x23: 2}
#         self.current_mode = 0
#         self.current_speed_level = 1 
#         self.profile_changed = True
#         self.profile_lock = threading.Lock()

#         # 4. 状态机与核心参数
#         self.state = 'IDLE' 
#         self.current_target_name = ""
#         self.is_waiting = False
#         self.mission_queue = []  # 核心调度队列
        
#         self.feedback_hex = 0x01 
#         self.Kp_yaw, self.Kd_yaw = 0.6, 0.3
#         self.prev_yaw_error = 0.0
        
#         self.track_start_pt = None  
#         self.track_end_pt = None    
#         self.track_speed = 0.15     
#         self.door_yaw = 0.0
#         self.exit_start_pose = None 

#         # 5. ROS 2 接口
#         self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 20)
#         self.profile_pub = self.create_publisher(Int32MultiArray, '/drive_profile', 1)
#         self.rviz_sub = self.create_subscription(PoseStamped, '/rviz_goal', self.rviz_goal_callback, 10)
#         self.tf_buffer = Buffer()
#         self.tf_listener = TransformListener(self.tf_buffer, self)
        
#         self.chassis_timer = self.create_timer(0.05, self.chassis_control_loop)       
#         self.profile_timer = self.create_timer(0.5, self.publish_profile_timer_cb)
        
#         self.read_thread = threading.Thread(target=self.serial_loop)
#         self.read_thread.daemon = True 
#         self.read_thread.start()
#         self.get_logger().info("[System] 异步语音监听守护线程已启动。")

#     # ==========================================
#     # 驾驶配置广播
#     # ==========================================
#     def publish_profile_timer_cb(self):
#         with self.profile_lock:
#             m = self.current_mode
#             s = self.current_speed_level
#             changed = self.profile_changed
#             self.profile_changed = False

#         msg = Int32MultiArray()
#         msg.data = [m, s]
#         self.profile_pub.publish(msg)

#         if changed:
#             self.get_logger().info(f"[Profile] 驾驶配置更新已发布: mode={m}, speed_level={s}")

#     # ==========================================
#     # 核心连招系统 (Mission Queue)
#     # ==========================================
#     def execute_next_mission(self):
#         if not self.mission_queue:
#             self.state = 'IDLE'
#             self.get_logger().info("🎉 [Mission] 拓扑导航连招全部执行完毕！")
            
#             if self.current_target_name == "厕所倒车模式":
#                 self.get_logger().info("[Docking] 抵达厕所，3秒后自动执行倒车泊车序列...")
#                 try: self.ser.write(bytes([self.feedback_hex]))
#                 except Exception: pass
                
#                 def start_docking():
#                     self.state = 'DOCKING'
#                     self.prev_yaw_error = 0.0
#                 threading.Timer(3.0, start_docking).start()
#             return
            
#         task = self.mission_queue.pop(0)
        
#         if task['type'] == 'nav':
#             self.state = 'NAVIGATING'
#             self.get_logger().info(f"[Mission] 发起 Nav2 去往: {task['goal']}")
#             self.nav2.send_goal(task['goal'])
            
#         elif task['type'] == 'door':
#             self.start_door_sequence(task['p1'], task['p2'])

#     def trigger_mission(self, room_hex, custom_goal=None):
#         if self.state not in ['IDLE', 'LOCKED']: 
#             self.get_logger().warn("[Mission] 系统繁忙，忽略新指令。")
#             return
        
#         pose = self.get_current_pose()
#         current_room_hex = self.db.identify_room_by_polygon(pose[0], pose[1]) if pose else None
            
#         self.get_logger().info(f"📍 当前坐标: {pose}, 识别出的房间ID: {current_room_hex}, 目标房间ID: {room_hex}")

#         target_info = self.db.topological_map[room_hex]
#         final_goal = custom_goal if custom_goal else target_info['default_goal']
#         self.current_target_name = target_info['name']
#         self.mission_queue = []

#         if current_room_hex and current_room_hex != room_hex:
#             curr_info = self.db.topological_map[current_room_hex]
#             if curr_info.get('is_room', False):
#                 door = curr_info['door']
#                 exit_yaw = door['enter_yaw'] + math.pi
#                 if exit_yaw > math.pi: exit_yaw -= 2 * math.pi
                
#                 self.mission_queue.append({'type': 'nav', 'goal': [door['inside_node'][0], door['inside_node'][1], exit_yaw]})
#                 self.mission_queue.append({'type': 'door', 'p1': door['inside_node'], 'p2': door['outside_node']})

#         if target_info.get('is_room', False):
#             door = target_info['door']
#             self.mission_queue.append({'type': 'nav', 'goal': [door['outside_node'][0], door['outside_node'][1], door['enter_yaw']]})
#             self.mission_queue.append({'type': 'door', 'p1': door['outside_node'], 'p2': door['inside_node']})

#         self.mission_queue.append({'type': 'nav', 'goal': final_goal})
        
#         self.get_logger().info(f"[Mission] 拓扑分析完成，生成了 {len(self.mission_queue)} 步组合连招。")
#         self.execute_next_mission()

#     def on_nav2_success(self):
#         self.execute_next_mission()

#     def on_nav2_fail(self):
#         self.state = 'IDLE'
#         self.mission_queue = [] 

#     def start_door_sequence(self, p1, p2, speed=0.15):
#         self.track_start_pt = p1
#         self.track_end_pt = p2
#         self.track_speed = speed
#         self.door_yaw = math.atan2(p2[1] - p1[1], p2[0] - p1[0])
#         self.prev_yaw_error = 0.0
#         self.state = 'ALIGNING_DOOR' 
#         self.get_logger().info(f"📐 [Control] 开始过门。第一步: 原地旋转对齐 ({self.door_yaw:.2f} rad)")

#     # ==========================================
#     # RViz 与 串口循环
#     # ==========================================
#     def rviz_goal_callback(self, msg):
#         click_x = msg.pose.position.x
#         click_y = msg.pose.position.y
#         q = msg.pose.orientation
#         siny_cosp = 2 * (q.w * q.z + q.x * q.y)
#         cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
#         click_yaw = math.atan2(siny_cosp, cosy_cosp)
        
#         room_hex = self.db.identify_room_by_polygon(click_x, click_y)
#         if room_hex:
#             self.trigger_mission(room_hex, custom_goal=[click_x, click_y, click_yaw])
#         else:
#             self.state = 'NAVIGATING'
#             self.mission_queue = [] 
#             self.nav2.send_goal([click_x, click_y, click_yaw])

#     def serial_loop(self):
#         while rclpy.ok():
#             try:
#                 if self.ser.in_waiting > 0:
#                     incoming_byte = self.ser.read(1)
#                     if not incoming_byte:
#                         continue
                    
#                     cmd_hex = ord(incoming_byte)
#                     if cmd_hex in [0xff, 0x00]:
#                         continue

#                     self.get_logger().info(f"[Serial] 接收到有效载荷 (Hex): {hex(cmd_hex)}")

#                     # =======================================================
#                     # 1. 动态标定指令解析 (0x40 - 0x45) 
#                     # 完全沿用原版录制位姿逻辑与二维码逻辑
#                     # =======================================================
#                     calibration_map = {
#                         0x40: "马桶", 
#                         0x41: "厕所", 
#                         0x42: "客厅", 
#                         0x43: "厨房", 
#                         0x44: "主卧", 
#                         0x45: "客卧"
#                     }
                    
#                     if cmd_hex in calibration_map:
#                         room_name = calibration_map[cmd_hex]
#                         self.get_logger().info(f"[Calibration] 收到标定指令: {room_name} ({hex(cmd_hex)})...")
                        
#                         if cmd_hex == 0x41:
#                             is_verified = self.verify_toilet_qr()
#                             if is_verified:
#                                 try:
#                                     self.ser.write(b'1')
#                                     self.get_logger().info("[Serial] 已向语音模块反馈文本 '1'")
#                                 except Exception as e:
#                                     self.get_logger().error(f"[Serial] 串口发送反馈失败: {e}")
#                             else:
#                                 try:
#                                     self.ser.write(b'0')
#                                     self.get_logger().info("[Serial] 已向语音模块反馈文本 '0' (验证失败)")
#                                 except Exception as e:
#                                     self.get_logger().error(f"[Serial] 串口发送反馈失败: {e}")
                                
#                                 self.ser.reset_input_buffer()
#                                 continue 

#                         current_pose = self.get_current_pose()
#                         if current_pose:
#                             # 建立 语音标定码 -> 拓扑地图房间码 的映射
#                             calib_to_nav_map = {
#                                 0x41: 0x35, # 厕所
#                                 0x42: 0x32, # 客厅
#                                 0x43: 0x33, # 厨房
#                                 0x44: 0x34, # 主卧
#                                 0x45: 0x36  # 客卧
#                             }

#                             if cmd_hex == 0x40:   
#                                 self.db.toilet_dock_target = current_pose
#                                 self.get_logger().info(f"✅ [标定成功] 马桶基准点坐标已更新: {current_pose}")
#                             elif cmd_hex in calib_to_nav_map: 
#                                 target_room_hex = calib_to_nav_map[cmd_hex]
#                                 if target_room_hex in self.db.topological_map:
#                                     self.db.topological_map[target_room_hex]['default_goal'] = current_pose
#                                     room_name = self.db.topological_map[target_room_hex]['name']
#                                     self.get_logger().info(f"✅ [标定成功] {room_name} 坐标已更新: {current_pose}")
                            
#                             self.db.save_waypoints() # 保存至硬盘
#                         else:
#                             self.get_logger().warn(f"❌ [标定失败] 请确认定位系统运行正常。")
                        
#                         self.ser.reset_input_buffer()
#                         continue

#                     # =======================================================
#                     # 2. 驶出解锁直行
#                     # =======================================================
#                     if cmd_hex == 0x30:
#                         if self.state in ['LOCKED', 'IDLE']:
#                             self.state = 'EXITING'
#                             self.exit_start_pose = None
#                             self.get_logger().info("[Trigger] 收到驶出指令(0x30)。轮椅即将直行 0.5 米并解锁导航。")
#                         else:
#                             self.get_logger().warn("[State] 轮椅正在运动中，忽略驶出指令。")
#                         self.ser.reset_input_buffer()
#                         continue

#                     # =======================================================
#                     # 3. 模式与速度配置指令解析
#                     # =======================================================
#                     if cmd_hex in self.mode_codes or cmd_hex in self.speed_codes:
#                         trigger_publish = False
#                         with self.profile_lock:
#                             if cmd_hex in self.mode_codes:
#                                 new_mode = self.mode_codes[cmd_hex]
#                                 if self.current_mode != new_mode:
#                                     self.current_mode = new_mode
#                                     self.profile_changed = True
                            
#                             if cmd_hex in self.speed_codes:
#                                 new_speed = self.speed_codes[cmd_hex]
#                                 if self.current_speed_level != new_speed:
#                                     self.current_speed_level = new_speed
#                                     self.profile_changed = True
                            
#                             if self.profile_changed:
#                                 trigger_publish = True
                        
#                         if trigger_publish:
#                             self.publish_profile_timer_cb()
#                         continue 

#                     # =======================================================
#                     # 4. 语音发起导航
#                     # =======================================================
#                     if cmd_hex in self.db.topological_map:
#                         if self.state == 'LOCKED':
#                             self.get_logger().warn("[State] 导航处于锁定状态！必须先下发驶出指令(0x30)解锁。")
#                             self.ser.reset_input_buffer()
#                             continue

#                         if self.state != 'IDLE':
#                             self.get_logger().warn("[State] 状态机处于非空闲态，当前指令已丢弃。")
#                             self.ser.reset_input_buffer()
#                             continue
                            
#                         if self.is_waiting:
#                             self.get_logger().warn("[State] 防抖机制已拦截重复指令。")
#                             self.ser.reset_input_buffer()
#                             continue

#                         self.is_waiting = True
#                         self.get_logger().info(f"[Trigger] 保护期起算(3.0s)...")
                        
#                         def execute_delayed_nav():
#                             self.trigger_mission(cmd_hex)
#                             self.is_waiting = False
#                             self.ser.reset_input_buffer() 

#                         timer = threading.Timer(3.0, execute_delayed_nav)
#                         timer.start()

#                     else:
#                         if cmd_hex not in self.mode_codes and cmd_hex not in self.speed_codes and not (0x40 <= cmd_hex <= 0x45) and cmd_hex != 0x30:
#                             self.get_logger().warn(f"[Parser] 未定义指令: {hex(cmd_hex)}。")

#             except Exception as e:
#                 self.get_logger().error(f"[Serial] 串行读取异常: {e}")
#                 time.sleep(1.0)

#     # ==========================================
#     # 工具与底盘 PID 闭环
#     # ==========================================
#     def get_current_pose(self):
#         try:
#             trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
#             x = trans.transform.translation.x
#             y = trans.transform.translation.y
#             q = trans.transform.rotation
#             siny_cosp = 2 * (q.w * q.z + q.x * q.y)
#             cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
#             return [round(x, 3), round(y, 3), round(math.atan2(siny_cosp, cosy_cosp), 3)]
#         except Exception as e: 
#             self.get_logger().error(f"[Calibration] 获取当前位姿失败，TF 树可能未就绪: {e}")
#             return None

#     def verify_toilet_qr(self):
#         self.get_logger().info("📷 正在启动摄像头，开始验证厕所专属二维码 (超时时间: 6秒)...")
#         cap = cv2.VideoCapture(0) 
#         if not cap.isOpened(): 
#             self.get_logger().error("❌ 无法打开摄像头！")
#             return False
            
#         cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
#         cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        
#         success_count = 0
#         target_count = 5
#         timeout = 6.0
#         start_time = time.time()
        
#         try:
#             while time.time() - start_time < timeout:
#                 ret, frame = cap.read()
#                 if not ret: 
#                     time.sleep(0.1)
#                     continue
#                 barcodes = pyzbar.decode(frame)
#                 for barcode in barcodes:
#                     barcode_data = barcode.data.decode("utf-8")
#                     if barcode_data == "1":
#                         success_count += 1
#                         self.get_logger().info(f"🔍 [QR] 成功识别目标文本 '1' ({success_count}/{target_count})")
#                         time.sleep(0.2) 
#                 if success_count >= target_count: 
#                     self.get_logger().info("✅ [QR] 验证成功！符合入厕标定条件。")
#                     return True
            
#             self.get_logger().warn("⚠️ [QR] 验证超时！在6秒内未连续5次扫描到内容为 '1' 的二维码。")
#             return False
#         finally:
#             cap.release()

#     def chassis_control_loop(self):
#         if self.state not in ['DOCKING', 'EXITING', 'CROSSING_DOOR']: return
#         pose = self.get_current_pose()
#         if not pose: return
#         current_x, current_y, current_yaw = pose[0], pose[1], pose[2]

#         # 1. 过门准备：原地对正
#         if self.state == 'ALIGNING_DOOR':
#             yaw_error = math.atan2(math.sin(self.door_yaw - current_yaw), math.cos(self.door_yaw - current_yaw))
#             if abs(yaw_error) < 0.05:
#                 self.stop_robot()
#                 self.prev_yaw_error = 0.0
#                 self.state = 'CROSSING_DOOR' 
#                 self.get_logger().info("✅ [Control] 对正完毕，切入循迹直行模式。")
#                 return
#             d_error = yaw_error - self.prev_yaw_error
#             angular_z = max(-0.5, min(0.5, (self.Kp_yaw * yaw_error) + (self.Kd_yaw * d_error)))
#             self.prev_yaw_error = yaw_error
#             self.cmd_pub.publish(self._make_twist(0.0, angular_z))
#             return

#         # 2. 虚拟循迹过门 (Stanley 算法)
#         if self.state == 'CROSSING_DOOR':
#             x1, y1 = self.track_start_pt[0], self.track_start_pt[1]
#             x2, y2 = self.track_end_pt[0], self.track_end_pt[1]
#             path_yaw = math.atan2(y2 - y1, x2 - x1)
#             dist_to_goal = math.hypot(current_x - x2, current_y - y2)
            
#             if dist_to_goal < 0.05:
#                 self.stop_robot()
#                 self.get_logger().info("✅ [Control] 成功穿过门框！")
#                 self.execute_next_mission() 
#                 return

#             dx_from_start, dy_from_start = current_x - x1, current_y - y1
#             e_lat = -dx_from_start * math.sin(path_yaw) + dy_from_start * math.cos(path_yaw)
#             yaw_correction = max(-0.5, min(0.5, math.atan(3.0 * e_lat)))
#             target_yaw_dynamic = path_yaw + yaw_correction
            
#             yaw_error = math.atan2(math.sin(target_yaw_dynamic - current_yaw), math.cos(target_yaw_dynamic - current_yaw))
#             if abs(yaw_error) < 0.02: yaw_error = 0.0
#             d_error = yaw_error - self.prev_yaw_error
#             angular_z = max(-0.4, min(0.4, (self.Kp_yaw * yaw_error) + (self.Kd_yaw * d_error)))
#             self.prev_yaw_error = yaw_error
#             self.cmd_pub.publish(self._make_twist(self.track_speed, angular_z))
#             return

#         # 3. 解锁直行
#         if self.state == 'EXITING':
#             if self.exit_start_pose is None: self.exit_start_pose = (current_x, current_y)
#             if math.hypot(current_x - self.exit_start_pose[0], current_y - self.exit_start_pose[1]) < 0.5:
#                 self.cmd_pub.publish(self._make_twist(0.15, 0.0))
#             else:
#                 self.stop_robot()
#                 self.state = 'IDLE'
#                 self.exit_start_pose = None
#                 self.get_logger().info("[Control] 直行完毕，导航机彻底解锁！")
#             return 

#         # 4. 厕所泊车倒退
#         if self.state == 'DOCKING':
#             target_x, target_y, target_yaw_base = self.db.toilet_dock_target
#             dx, dy = current_x - target_x, current_y - target_y
#             distance = math.hypot(dx, dy)
#             e_lat = -dx * math.sin(target_yaw_base) + dy * math.cos(target_yaw_base)
#             e_lon = dx * math.cos(target_yaw_base) + dy * math.sin(target_yaw_base)

#             if e_lon < 0.01 or distance < 0.02:
#                 self.stop_robot()
#                 self.state = 'LOCKED' 
#                 self.get_logger().info(f"[Control] 泊车序列完毕！导航已被锁定。(剩余纵向: {e_lon:.3f}m)")
#                 return

#             yaw_correction = max(-0.6, min(0.6, math.atan(2.0 * e_lat)))
#             target_yaw_dynamic = target_yaw_base + yaw_correction
#             yaw_error = math.atan2(math.sin(target_yaw_dynamic - current_yaw), math.cos(target_yaw_dynamic - current_yaw))
#             if abs(yaw_error) < 0.03: yaw_error = 0.0
#             d_error = yaw_error - self.prev_yaw_error
#             angular_z = max(-0.4, min(0.4, (self.Kp_yaw * yaw_error) + (self.Kd_yaw * d_error)))
#             self.prev_yaw_error = yaw_error

#             linear_x = -0.15 
#             if distance < 0.40:
#                 linear_x = -0.04 - (distance / 0.40) * 0.11
#                 angular_z = angular_z * (distance / 0.40)
#             self.cmd_pub.publish(self._make_twist(linear_x, angular_z))

#     def stop_robot(self):
#         self.cmd_pub.publish(self._make_twist(0.0, 0.0))

#     def _make_twist(self, linear, angular):
#         cmd = Twist()
#         cmd.linear.x = float(linear)
#         cmd.angular.z = float(angular)
#         return cmd

# def main(args=None):
#     rclpy.init(args=args)
#     node = VoiceNavBridge()
#     try: rclpy.spin(node)
#     except KeyboardInterrupt: pass
#     finally:
#         node.stop_robot()
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()


#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
import math
import threading
import time
import cv2
from pyzbar import pyzbar

from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Int32MultiArray
from tf2_ros import Buffer, TransformListener

# 导入解耦的模块
from jt_chair.waypoint_manager import WaypointManager
from jt_chair.nav2_commander import Nav2Commander

class VoiceNavBridge(Node):
    def __init__(self):
        super().__init__('voice_nav_bridge')

        # 1. 实例化数据层和规划层
        config_path = '/home/orangepi/slam_ws/src/jt_chair/config/waypoints_config.json'
        self.db = WaypointManager(self.get_logger(), config_path)
        self.nav2 = Nav2Commander(self, self.on_nav2_success, self.on_nav2_fail)

        # 2. 硬件串口配置
        self.serial_port = '/dev/ttyS4'
        self.baud_rate = 115200
        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=0.5)
            self.get_logger().info(f"[Hardware] 串口通信链路已建立: {self.serial_port} (Baud: {self.baud_rate})")
        except Exception as e:
            self.get_logger().error(f"[Hardware] 串口初始化失败，请验证硬件连线与系统权限: {e}")
            return

        # 3. 驾驶模式与速度控制指令映射 
        self.mode_codes = {0x11: 0, 0x12: 1, 0x13: 2}
        self.speed_codes = {0x21: 0, 0x22: 1, 0x23: 2}
        self.current_mode = 0
        self.current_speed_level = 1 
        self.profile_changed = True
        self.profile_lock = threading.Lock()

        # 4. 状态机与核心参数
        self.state = 'IDLE' 
        self.current_target_name = ""
        self.is_waiting = False
        self.mission_queue = []  # 核心调度队列
        
        self.feedback_hex = 0x01 
        self.Kp_yaw, self.Kd_yaw = 0.6, 0.3
        self.prev_yaw_error = 0.0
        
        self.track_start_pt = None  
        self.track_end_pt = None    
        self.track_speed = 0.15     
        self.door_yaw = 0.0
        self.exit_start_pose = None
        self.exit_start_yaw = None       # 驶出直行的起始朝向（航向锁定用）

        # ---- 过门循迹专用参数 ----
        self.door_start_time = None       # 过门超时计时起点
        self.door_timeout = 15.0          # 过门最大允许时间（秒）
        self.door_line_p1 = None          # 门线起点（存储航点）
        self.door_line_p2 = None          # 门线终点（存储航点）
        self.door_line_len = 0.0          # 门线总长度
        self.door_line_ux = 0.0           # 门线单位向量 x
        self.door_line_uy = 0.0           # 门线单位向量 y
        self.yaw_trim = 0.0               # 偏航修正量 (rad): >0 左转补偿, 抵消车体右偏

        # 5. ROS 2 接口
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 20)
        self.profile_pub = self.create_publisher(Int32MultiArray, '/drive_profile', 1)
        self.rviz_sub = self.create_subscription(PoseStamped, '/rviz_goal', self.rviz_goal_callback, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.chassis_timer = self.create_timer(0.05, self.chassis_control_loop)       
        self.profile_timer = self.create_timer(0.5, self.publish_profile_timer_cb)
        
        self.read_thread = threading.Thread(target=self.serial_loop)
        self.read_thread.daemon = True 
        self.read_thread.start()
        self.get_logger().info("[System] 🎯 自动驾驶调度核心启动完毕！")

    # ==========================================
    # 驾驶配置广播
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

    # ==========================================
    # 核心任务流水线 (Mission Queue)
    # ==========================================
    def execute_next_mission(self):
        """严格顺序消耗队列任务"""
        if not self.mission_queue:
            self.state = 'IDLE'
            self.get_logger().info("🎉 [Mission] 拓扑导航当前阶段所有连招执行完毕！")
            
            # 【时序卡点】：只有当所有队列走完（意味着已经到达了厕所的目标位姿），才触发最后的倒车马桶动作
            if self.current_target_name == "厕所倒车模式":
                self.get_logger().info("[Docking] 已经安全到达厕所目标点，3秒后接管底盘执行马桶倒车序列...")
                try: self.ser.write(bytes([self.feedback_hex]))
                except Exception: pass
                
                def start_docking():
                    self.state = 'DOCKING'
                    self.prev_yaw_error = 0.0
                threading.Timer(3.0, start_docking).start()
            return
            
        task = self.mission_queue.pop(0)
        
        if task['type'] == 'nav':
            self.state = 'NAVIGATING'
            self.get_logger().info(f"[Mission] 激活 Nav2 点对点规划 -> 目标: {task['goal']}")
            self.nav2.send_goal(task['goal'])
            
        elif task['type'] == 'door':
            self.start_door_sequence(task['p1'], task['p2'])

    def trigger_mission(self, room_hex, custom_goal=None):
        """根据当前空间位置，严格建立时序流水线"""
        if self.state not in ['IDLE', 'LOCKED']: 
            self.get_logger().warn("[Mission] 系统繁忙，忽略新指令。")
            return
        
        pose = self.get_current_pose()
        current_room_hex = self.db.identify_room_by_polygon(pose[0], pose[1]) if pose else None
            
        target_info = self.db.topological_map[room_hex]
        final_goal = custom_goal if custom_goal else target_info['default_goal']
        self.current_target_name = target_info['name']
        self.mission_queue = []

        self.get_logger().info(f"📍 [Debug] 当前位置: {pose}, 识别房间ID: {current_room_hex}, 目标房间ID: {room_hex}")

        # 兼容性判定：如果当前在厕所区域内调用0x31或0x35，一律视为同房间
        is_sim_room = (current_room_hex == room_hex) or (room_hex in [0x31, 0x35] and current_room_hex in [0x31, 0x35])

        if is_sim_room:
            # 流程 A：如果已经在目标区域内，跳过一切过门动作，直接 Nav2 移动到 JSON 记忆的目标点
            self.get_logger().info("[Mission] 判定为房间内本地微调，直接拉起最终点规划。")
            self.mission_queue.append({'type': 'nav', 'goal': final_goal})
        else:
            # 流程 B：跨区域导航，严格生成【出门 -> 过门 -> 进门 -> 目标点】时序
            
            # Step 1: 出门阶段（若当前身处带门的房间内）
            if current_room_hex and current_room_hex in self.db.topological_map:
                curr_info = self.db.topological_map[current_room_hex]
                if curr_info.get('is_room', False):
                    door = curr_info['door']
                    exit_yaw = door['enter_yaw'] + math.pi
                    if exit_yaw > math.pi: exit_yaw -= 2 * math.pi
                    
                    # 检查是否有出门回退距离（给 Stanley 留出对齐空间）
                    exit_offset = door.get('exit_offset', 0.0)
                    if exit_offset > 0.0:
                        ux = math.cos(exit_yaw)
                        uy = math.sin(exit_yaw)
                        p1_back = [
                            door['inside_node'][0] - ux * exit_offset,
                            door['inside_node'][1] - uy * exit_offset,
                        ]
                        self.get_logger().info(
                            f"[Queue] 出门回退 {exit_offset:.1f}m "
                            f"({p1_back[0]:.3f}, {p1_back[1]:.3f})")
                    else:
                        p1_back = list(door['inside_node'])

                    self.mission_queue.append({'type': 'nav', 'goal': [p1_back[0], p1_back[1], exit_yaw]})
                    self.get_logger().info(f"[Queue] 插入连招: 关闭导航，虚拟循迹直行至【门外准备点】")
                    self.mission_queue.append({'type': 'door', 'p1': p1_back, 'p2': door['outside_node']})

            # Step 2: 进门阶段（若目标是一个独立的封闭房间）
            if target_info.get('is_room', False):
                door = target_info['door']
                # 检查是否有进门助跑距离（给 Stanley 留出对齐空间，对称于出门的 exit_offset）
                approach_offset = door.get('approach_offset', 0.0)
                enter_yaw = door['enter_yaw']
                if approach_offset > 0.0:
                    ux = math.cos(enter_yaw)
                    uy = math.sin(enter_yaw)
                    p0_approach = [
                        door['outside_node'][0] - ux * approach_offset,
                        door['outside_node'][1] - uy * approach_offset,
                    ]
                    self.get_logger().info(
                        f"[Queue] 进门助跑 {approach_offset:.1f}m "
                        f"({p0_approach[0]:.3f}, {p0_approach[1]:.3f})")
                else:
                    p0_approach = list(door['outside_node'])

                self.get_logger().info(f"[Queue] 插入连招: 导航至目标房间【门外准备点】")
                self.mission_queue.append({'type': 'nav', 'goal': [p0_approach[0], p0_approach[1], enter_yaw]})
                self.get_logger().info(f"[Queue] 插入连招: 关闭导航，虚拟循迹直行至【门内准备点】")
                self.mission_queue.append({'type': 'door', 'p1': p0_approach, 'p2': door['inside_node']})

            # Step 3: 房内深处导航阶段（从门内准备点，使用 Nav2 最终规划至 JSON 文件的目标点）
            self.get_logger().info(f"[Queue] 插入连招: 最终从门内点导航至【JSON目标位姿】")
            self.mission_queue.append({'type': 'nav', 'goal': final_goal})
        
        self.get_logger().info(f"[Mission] 拓扑解算成功，共生成 {len(self.mission_queue)} 步标准串行任务。")
        self.execute_next_mission()

    def on_nav2_success(self):
        """当前段 Nav2 任务成功后，自动触发下一流水线"""
        self.execute_next_mission()

    def on_nav2_fail(self):
        self.state = 'IDLE'
        self.mission_queue = [] 

    def start_door_sequence(self, p1, p2, speed=0.2):
        """过门：将当前位置投影到门线上，启动平滑循迹过门"""
        self.door_line_p1 = p1
        self.door_line_p2 = p2
        self.track_start_pt = list(p1)
        self.track_end_pt = list(p2)
        self.track_speed = speed
        self.prev_yaw_error = 0.0
        self.door_start_time = time.monotonic()

        # 计算门线参数（方向单位向量 + 总长）
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        self.door_line_len = math.hypot(dx, dy)
        if self.door_line_len > 0.001:
            self.door_line_ux = dx / self.door_line_len
            self.door_line_uy = dy / self.door_line_len
        else:
            self.door_line_ux = 1.0
            self.door_line_uy = 0.0

        self.door_yaw = math.atan2(dy, dx)

        # 将当前位置投影到门线上作为虚拟起点（消除 Nav2 停位误差）
        pose = self.get_current_pose()
        if pose and self.door_line_len > 0.001:
            vx = pose[0] - p1[0]
            vy = pose[1] - p1[1]
            proj = vx * self.door_line_ux + vy * self.door_line_uy
            proj = max(0.0, min(self.door_line_len, proj))
            self.track_start_pt = [
                p1[0] + proj * self.door_line_ux,
                p1[1] + proj * self.door_line_uy,
            ]
            actual_remain = self.door_line_len - proj
        else:
            actual_remain = self.door_line_len

        # 注：不原地对正，轮椅需要前行才能有效转向，
        # Stanley 控制器会在前进中自然完成朝向修正
        self.state = 'CROSSING_DOOR'

        # 详细诊断日志
        door_deg = math.degrees(self.door_yaw)
        if pose:
            current_to_door = math.degrees(math.atan2(math.sin(self.door_yaw - pose[2]),
                                                       math.cos(self.door_yaw - pose[2])))
            warn_flag = " ⚠️ 偏角过大！" if abs(current_to_door) > 30 else ""
            self.get_logger().info(
                f"📐 [Control] 过门开始: "
                f"p1=({p1[0]:.2f},{p1[1]:.2f}) → p2=({p2[0]:.2f},{p2[1]:.2f}), "
                f"方向={door_deg:.1f}°, 线长={self.door_line_len:.2f}m, "
                f"车头偏差={current_to_door:.1f}°{warn_flag}")
        else:
            self.get_logger().info(
                f"📐 [Control] 过门开始: "
                f"p1=({p1[0]:.2f},{p1[1]:.2f}) → p2=({p2[0]:.2f},{p2[1]:.2f}), "
                f"方向={door_deg:.1f}°, 线长={self.door_line_len:.2f}m")

    # ==========================================
    # 交互回调与读取循环
    # ==========================================
    def rviz_goal_callback(self, msg):
        click_x = msg.pose.position.x
        click_y = msg.pose.position.y
        q = msg.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        click_yaw = math.atan2(siny_cosp, cosy_cosp)
        
        room_hex = self.db.identify_room_by_polygon(click_x, click_y)
        if room_hex:
            self.trigger_mission(room_hex, custom_goal=[click_x, click_y, click_yaw])
        else:
            self.state = 'NAVIGATING'
            self.mission_queue = [] 
            self.nav2.send_goal([click_x, click_y, click_yaw])

    def serial_loop(self):
        self.get_logger().info("[Serial] 串口读取线程已进入监听状态...")
        while rclpy.ok():
            try:
                if not self.ser.is_open:
                    self.get_logger().error("[Serial] 串口意外关闭！")
                    break
                    
                if self.ser.in_waiting == 0:
                    time.sleep(0.05) 
                    continue

                incoming_byte = self.ser.read(1)
                cmd_hex = ord(incoming_byte)
                
                if cmd_hex in [0xff, 0x00]:
                    continue

                self.get_logger().info(f"[Serial] 接收到有效载荷 (Hex): {hex(cmd_hex)}")

                # =======================================================
                # 1. 动态标定指令解析 (0x40 - 0x45)
                # =======================================================
                calibration_map = {
                    0x40: "马桶", 0x41: "厕所", 0x42: "客厅", 
                    0x43: "厨房", 0x44: "主卧", 0x45: "客卧"
                }
                
                if cmd_hex in calibration_map:
                    room_name = calibration_map[cmd_hex]
                    self.get_logger().info(f"[Calibration] 收到标定指令: {room_name} ({hex(cmd_hex)})...")
                    
                    if cmd_hex == 0x41:
                        is_verified = self.verify_toilet_qr()
                        if is_verified:
                            try:
                                self.ser.write(b'1')
                                self.get_logger().info("[Serial] 已向语音模块反馈文本 '1'")
                            except Exception as e:
                                self.get_logger().error(f"[Serial] 串口发送反馈失败: {e}")
                        else:
                            try:
                                self.ser.write(b'0')
                                self.get_logger().info("[Serial] 已向语音模块反馈文本 '0' (验证失败)")
                            except Exception as e:
                                self.get_logger().error(f"[Serial] 串口发送反馈失败: {e}")
                            self.ser.reset_input_buffer()
                            continue 

                    current_pose = self.get_current_pose()
                    if current_pose:
                        calib_to_nav_map = {
                            0x41: 0x31, # 对应到厕所的 0x31
                            0x42: 0x32, # 客厅
                            0x43: 0x33, # 厨房
                            0x44: 0x34, # 主卧
                            0x45: 0x36  # 客卧
                        }

                        if cmd_hex == 0x40:   
                            self.db.toilet_dock_target = current_pose
                            self.get_logger().info(f"✅ [标定成功] 马桶基准点坐标已更新: {current_pose}")
                        elif cmd_hex in calib_to_nav_map: 
                            target_room_hex = calib_to_nav_map[cmd_hex]
                            if target_room_hex == 0x31:
                                self.db.topological_map[0x31]['default_goal'] = current_pose
                                self.db.topological_map[0x35]['default_goal'] = current_pose
                            elif target_room_hex in self.db.topological_map:
                                self.db.topological_map[target_room_hex]['default_goal'] = current_pose
                                
                            room_name = self.db.topological_map[target_room_hex]['name']
                            self.get_logger().info(f"✅ [标定成功] {room_name} 坐标已更新: {current_pose}")
                        
                        self.db.save_waypoints()
                    else:
                        self.get_logger().warn(f"❌ [标定失败] 请确认定位系统运行正常。")
                    
                    self.ser.reset_input_buffer()
                    continue

                # =======================================================
                # 2. 驶出解锁直行
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
                if cmd_hex in self.db.topological_map:
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

                    self.is_waiting = True
                    self.get_logger().info(f"[Trigger] 保护期起算(3.0s)...")
                    
                    def execute_delayed_nav():
                        self.trigger_mission(cmd_hex)
                        self.is_waiting = False
                        self.ser.reset_input_buffer() 

                    timer = threading.Timer(3.0, execute_delayed_nav)
                    timer.start()

                else:
                    if cmd_hex not in self.mode_codes and cmd_hex not in self.speed_codes and not (0x40 <= cmd_hex <= 0x45) and cmd_hex != 0x30:
                        self.get_logger().warn(f"[Parser] 未定义指令: {hex(cmd_hex)}。")

            except Exception as e:
                self.get_logger().error(f"[Serial] 串行读取异常: {e}")
                time.sleep(1.0)

    # ==========================================
    # 工具与底盘 PID 闭环
    # ==========================================
    def get_current_pose(self):
        try:
            trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0))
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            q = trans.transform.rotation
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            return [round(x, 3), round(y, 3), round(math.atan2(siny_cosp, cosy_cosp), 3)]
        except Exception as e: 
            self.get_logger().error(f"[Calibration] 获取当前位姿失败，TF 树可能未就绪: {e}")
            return None

    def verify_toilet_qr(self):
        self.get_logger().info("📷 正在启动摄像头，开始验证厕所专属二维码 (超时时间: 6秒)...")
        cap = None
        for i in [0, 11, 1, 12, 2]: 
            temp_cap = cv2.VideoCapture(i)
            if temp_cap.isOpened():
                ret, _ = temp_cap.read()
                if ret: 
                    cap = temp_cap
                    self.get_logger().info(f"✅ 成功找到可用摄像头节点: /dev/video{i}")
                    break
                temp_cap.release()
                
        if cap is None:
            self.get_logger().error("❌ 无法打开摄像头！")
            return False
            
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        
        success_count = 0
        target_count = 5
        start_time = time.time()
        
        try:
            while time.time() - start_time < 6.0:
                ret, frame = cap.read()
                if not ret: 
                    time.sleep(0.1)
                    continue
                barcodes = pyzbar.decode(frame)
                for barcode in barcodes:
                    if barcode.data.decode("utf-8") == "1":
                        success_count += 1
                        self.get_logger().info(f"🔍 [QR] 成功识别目标文本 '1' ({success_count}/{target_count})")
                        time.sleep(0.2) 
                if success_count >= target_count: 
                    self.get_logger().info("✅ [QR] 验证成功！符合入厕标定条件。")
                    return True
            return False
        finally:
            cap.release()

    def chassis_control_loop(self):
        if self.state not in ['DOCKING', 'EXITING', 'CROSSING_DOOR']: return
        pose = self.get_current_pose()
        if not pose: return
        current_x, current_y, current_yaw = pose[0], pose[1], pose[2]

        # 1. 虚拟循迹过门（参考DOCKING倒车结构，正向行走）
        if self.state == 'CROSSING_DOOR':
            x2, y2 = self.door_line_p2      # 目标点（门另一侧）
            path_yaw = self.door_yaw        # 门线朝向

            # ---- 在门线坐标系下分解（同DOCKING倒车逻辑） ----
            dx = current_x - x2      # 从目标指向机器人
            dy = current_y - y2
            distance = math.hypot(dx, dy)
            e_lat = -dx * math.sin(path_yaw) + dy * math.cos(path_yaw)    # 横向偏差
            e_lon = dx * math.cos(path_yaw) + dy * math.sin(path_yaw)     # 纵向进度

            # 到达判断：过了目标点或已靠近
            if e_lon >= 0.0 or distance < 0.10:
                self.stop_robot()
                self.get_logger().info("✅ [Control] 成功穿过门框！")
                self.execute_next_mission()
                return

            # ---- 超时保护 ----
            if self.door_start_time and time.monotonic() - self.door_start_time > self.door_timeout:
                self.stop_robot()
                self.get_logger().warning("⚠️ [Control] 过门超时(>15s)，强制跳过！")
                self.execute_next_mission()
                return

            # ---- 横向偏差→航向修正（同DOCKING结构，正向行走修正取反） ----
            yaw_correction = max(-0.3, min(0.3, math.atan(1.5 * e_lat)))
            target_yaw = path_yaw - yaw_correction    # 正向行走：偏左则右转

            yaw_error = math.atan2(math.sin(target_yaw - current_yaw),
                                    math.cos(target_yaw - current_yaw))
            if abs(yaw_error) < 0.02:
                yaw_error = 0.0
            d_error = yaw_error - self.prev_yaw_error
            angular_z = max(-0.4, min(0.4, (self.Kp_yaw * yaw_error) + (self.Kd_yaw * d_error) + self.yaw_trim))
            self.prev_yaw_error = yaw_error

            # ---- 速度：接近目标时减速（同DOCKING策略） ----
            if distance < 0.40:
                linear_x = 0.04 + (distance / 0.40) * (self.track_speed - 0.04)
                angular_z = angular_z * (distance / 0.40)
            else:
                linear_x = self.track_speed

            self.cmd_pub.publish(self._make_twist(linear_x, angular_z))
            return

        # 2. 驶出解锁直行（带 PD 航向保持）
        if self.state == 'EXITING':
            if self.exit_start_pose is None:
                self.exit_start_pose = (current_x, current_y)
                self.exit_start_yaw = current_yaw
                self.prev_yaw_error = 0.0

            # PD 航向锁定
            yaw_error = math.atan2(math.sin(self.exit_start_yaw - current_yaw),
                                    math.cos(self.exit_start_yaw - current_yaw))
            d_error = yaw_error - self.prev_yaw_error
            angular_z = max(-0.3, min(0.3, self.Kp_yaw * yaw_error + self.Kd_yaw * d_error))
            self.prev_yaw_error = yaw_error

            if math.hypot(current_x - self.exit_start_pose[0],
                          current_y - self.exit_start_pose[1]) < 0.5:
                self.cmd_pub.publish(self._make_twist(0.15, angular_z))
            else:
                self.stop_robot()
                self.state = 'IDLE'
                self.exit_start_pose = None
                self.exit_start_yaw = None
                self.get_logger().info("[Control] 直行完毕，导航机彻底解锁！")
            return

        # 3. 厕所泊车倒退
        if self.state == 'DOCKING':
            target_x, target_y, target_yaw_base = self.db.toilet_dock_target
            dx, dy = current_x - target_x, current_y - target_y
            distance = math.hypot(dx, dy)
            e_lat = -dx * math.sin(target_yaw_base) + dy * math.cos(target_yaw_base)
            e_lon = dx * math.cos(target_yaw_base) + dy * math.sin(target_yaw_base)

            if e_lon < 0.01 or distance < 0.02:
                self.stop_robot()
                self.state = 'LOCKED' 
                self.get_logger().info(f"[Control] 泊车序列完毕！导航已被锁定。(剩余纵向: {e_lon:.3f}m)")
                return

            yaw_correction = max(-0.6, min(0.6, math.atan(2.0 * e_lat)))
            target_yaw_dynamic = target_yaw_base + yaw_correction
            yaw_error = math.atan2(math.sin(target_yaw_dynamic - current_yaw), math.cos(target_yaw_dynamic - current_yaw))
            if abs(yaw_error) < 0.03: yaw_error = 0.0
            d_error = yaw_error - self.prev_yaw_error
            angular_z = max(-0.4, min(0.4, (self.Kp_yaw * yaw_error) + (self.Kd_yaw * d_error)))
            self.prev_yaw_error = yaw_error

            linear_x = -0.15 
            if distance < 0.40:
                linear_x = -0.04 - (distance / 0.40) * 0.11
                angular_z = angular_z * (distance / 0.40)
            self.cmd_pub.publish(self._make_twist(linear_x, angular_z))

    def stop_robot(self):
        self.cmd_pub.publish(self._make_twist(0.0, 0.0))

    def _make_twist(self, linear, angular):
        cmd = Twist()
        cmd.linear.x = float(linear)
        cmd.angular.z = float(angular)
        return cmd

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