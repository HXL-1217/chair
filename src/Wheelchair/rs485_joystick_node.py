#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import time

class SMC30RS485Joystick(Node):
    def __init__(self):
        super().__init__('smc30_joystick_node')
        
        # 串口参数配置
        self.declare_parameter('port', '/dev/ttyUSB1')
        self.declare_parameter('baudrate', 9600)  
        
        port_name = self.get_parameter('port').get_parameter_value().string_value
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        
        try:
            self.serial_port = serial.Serial(port_name, baudrate, timeout=0.02)
            self.get_logger().info(f"成功连接摇杆串口: {port_name} @ {baudrate}")
        except Exception as e:
            self.get_logger().error(f"无法打开串口 {port_name}: {e}")
            raise e

        # ROS 2 发布者
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # 定时器频率 (50Hz，匹配摇杆 20ms 的发送频率)
        self.timer = self.create_timer(0.02, self.timer_callback)
        
        # 状态控制变量
        self.mode = 1  
        self.last_button_state = 0
        
        # 速度限制
        self.max_linear_speed = 0.5   # m/s
        self.max_angular_speed = 1.0  # rad/s

    def timer_callback(self):
        try:
            # 检查缓冲区里堆积了多少数据
            bytes_waiting = self.serial_port.in_waiting
            
            if bytes_waiting >= 9:
                buffer = self.serial_port.read(bytes_waiting)
                latest_valid_frame = None
                
                # 寻找最新的一帧有效数据 (长度9，包头FF)
                for i in range(len(buffer) - 8):
                    if buffer[i] == 0xFF:
                        frame = buffer[i:i+9]
                        checksum = sum(frame[1:8]) & 0xFF
                        if checksum == frame[8]:
                            latest_valid_frame = frame
                            
                if latest_valid_frame:
                    self.parse_and_publish(latest_valid_frame)
                    
        except OSError as e:
            # 捕获物理断开异常 (Errno 5 等)
            self.get_logger().error(f"摇杆物理连接断开！(Input/output error)")
            
            # 【安全机制】立刻发送速度为 0 的指令，让轮椅紧急刹车！
            stop_msg = Twist()
            self.publisher_.publish(stop_msg)
            self.get_logger().error("已发送紧急刹车指令！节点即将退出，请检查USB线缆。")
            
            # 关闭定时器并退出节点
            self.timer.cancel()
            raise SystemExit
            
        except Exception as e:
            self.get_logger().error(f"读取串口时发生未知错误: {e}")

    def parse_and_publish(self, frame):
        # 解析 9 字节协议：FF YYH YYL XXH XXL ZZH ZZL Button CH
        
        # 提取 Y 轴数据 (第1、2字节)
        y_raw = (frame[1] << 8) | frame[2]
        # 提取 X 轴数据 (第3、4字节)
        x_raw = (frame[3] << 8) | frame[4]
        
        # 提取按钮状态 (第7字节)
        current_button = 1 if (frame[7] & 0x20) else 0
        
        # 模式切换逻辑 (松开到按下触发)
        if current_button == 1 and self.last_button_state == 0:
            self.mode += 1
            if self.mode > 3:
                self.mode = 1
            self.get_logger().info(f">>> 当前控制模式切换为: {self.mode} <<<")
        self.last_button_state = current_button
        
        # 数据归一化处理 (-1.0 到 1.0)
        x_norm = self.normalize_axis(x_raw)
        y_norm = self.normalize_axis(y_raw)
        
        # 创建 Twist 消息
        msg = Twist()
        
        # Y 轴统一控制前后
        # 手册中 Y轴: 上=0x03E0(992), 中心=0x0200(512), 下=0x0020(32)
        # 向上推为正速度
        msg.linear.x = float(y_norm * self.max_linear_speed)
        
        # 摇杆横向 X 轴控制逻辑
        # 手册中 X轴: 右=0x03E0(992), 中心=0x0200(512), 左=0x0020(32)
        # ROS 坐标系：+y向左平移，+z逆时针旋转。向右推摇杆取负适应坐标系。
        if self.mode == 1:
            msg.linear.y = 0.0
            msg.angular.z = float(-x_norm * self.max_angular_speed)
        elif self.mode == 2:
            msg.linear.y = float(-x_norm * self.max_linear_speed)
            msg.angular.z = 0.0
        elif self.mode == 3:
            msg.linear.x = 0.0
            msg.linear.y = 0.0
            msg.angular.z = float(-x_norm * self.max_angular_speed)

        self.publisher_.publish(msg)

    def normalize_axis(self, raw_val):
        # 中心点与上下限参数
        center = 512.0  
        max_val = 992.0 
        min_val = 32.0  
        
        if raw_val > max_val: raw_val = max_val
        if raw_val < min_val: raw_val = min_val
        
        if raw_val >= center:
            norm = (raw_val - center) / (max_val - center)
        else:
            norm = (raw_val - center) / (center - min_val)
            
        # 5% 的死区处理防止抖动
        if abs(norm) < 0.05:
            return 0.0
        return norm

def main(args=None):
    rclpy.init(args=args)
    node = SMC30RS485Joystick()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n[INFO] [smc30_joystick_node]: 正在退出...")
    finally:
        if hasattr(node, 'serial_port') and node.serial_port.is_open:
            node.serial_port.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
            print("[INFO] [smc30_joystick_node]: 已退出")

if __name__ == '__main__':
    main()