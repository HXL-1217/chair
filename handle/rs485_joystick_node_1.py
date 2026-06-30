#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
import serial
import time
import threading


class ModbusAnalogJoystick(Node):
    def __init__(self):
        super().__init__('modbus_joystick_node')
        
        # 串口参数配置 (根据手册，模块默认 9600, 8N1)
        self.declare_parameter('port', '/dev/ttyUSB1')
        self.declare_parameter('baudrate', 9600)  
        
        port_name = self.get_parameter('port').get_parameter_value().string_value
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        
        try:
            # timeout 设为 0.05s，适应 9600 波特率下的 Modbus 响应延迟
            self.serial_port = serial.Serial(port_name, baudrate, timeout=0.05)
            self.get_logger().info(f"成功连接 RS485 模块: {port_name} @ {baudrate}")
        except Exception as e:
            self.get_logger().error(f"无法打开串口 {port_name}: {e}")
            raise e

        # ROS 2 发布者
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 20)
        
        # 创建 Transient Local 的 QoS 策略，匹配语音节点的发布者
        qos_profile = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        
        # 订阅语音模块发出的 /drive_profile
        self.profile_sub = self.create_subscription(
            Int32MultiArray,
            '/drive_profile',
            self.profile_callback,
            qos_profile
        )
        
        # 配置状态变量及线程锁
        self.mode = 0          # 默认 0: 正常模式
        self.speed_level = 1   # 默认 1: 中速档
        self._profile_lock = threading.Lock()
        
        # 三档速度倍率，对应 speed_level 0, 1, 2
        self.speed_scale = [0.4, 0.7, 1.0]
        
        # 基础速度限制
        self.max_linear_speed = 1.5   # m/s (前后与横移复用此最大速度)
        self.max_angular_speed = 1.0  # rad/s

        # 定时器频率 (20Hz = 50ms)
        self.timer = self.create_timer(0.05, self.timer_callback)
        
        # 预先计算好的 Modbus 读取指令 (读取从站01的 0000H 和 0001H 两个寄存器)
        self.modbus_request = bytes([0x01, 0x04, 0x00, 0x00, 0x00, 0x02, 0x71, 0xCB])

    def profile_callback(self, msg: Int32MultiArray):
        """
        处理接收到的速度与模式切换指令
        """
        if len(msg.data) < 2:
            return
            
        m = int(msg.data[0])
        s = int(msg.data[1])
        
        # 越界保护
        m = max(0, min(2, m))
        s = max(0, min(2, s))
        
        with self._profile_lock:
            changed = (m != self.mode) or (s != self.speed_level)
            self.mode = m
            self.speed_level = s
            
        if changed:
            self.get_logger().info(f"接收到语音切换配置: mode={m}, speed_level={s} (倍率: {self.speed_scale[s]}x)")

    def timer_callback(self):
        try:
            # 1. 清空缓冲区
            self.serial_port.reset_input_buffer()
            
            # 2. 发送读取请求
            self.serial_port.write(self.modbus_request)
            
            # 3. 读取响应
            response = self.serial_port.read(9)
            
            if len(response) == 9:
                # 4. CRC16 校验
                if self.check_crc16(response):
                    self.parse_and_publish(response)
                else:
                    self.get_logger().warn("Modbus 数据帧 CRC 校验失败，丢弃该帧。")
            else:
                self.get_logger().warn(f"Modbus 响应超时或不完整，仅收到 {len(response)} 字节。")
                
        except OSError as e:
            self.get_logger().error("RS485 模块物理连接断开！紧急刹车！")
            self.publisher_.publish(Twist()) # 发送停止指令
            self.timer.cancel()
            raise SystemExit
        except Exception as e:
            self.get_logger().error(f"发生未知错误: {e}")

    def parse_and_publish(self, frame):
        # 提取 CH1 和 CH2 的原始寄存器值
        ch1_raw = (frame[3] << 8) | frame[4]
        ch2_raw = (frame[5] << 8) | frame[6]
        
        # 解析电压值
        voltage_y = self.parse_zs_voltage(ch1_raw) # 假设 CH1 接 Y轴 (前后)
        voltage_x = self.parse_zs_voltage(ch2_raw) # 假设 CH2 接 X轴 (左右)
        
        # 将 0~5V 电压归一化为 -1.0 ~ 1.0
        norm_x = self.normalize_voltage(voltage_y)
        norm_y = self.normalize_voltage(voltage_x)
        
        # 提取当前的模式与档位
        with self._profile_lock:
            current_mode = self.mode
            current_speed_level = self.speed_level
            
        # 计算当前档位的速度倍率
        scale = self.speed_scale[current_speed_level]
        
        # ========================================================
        # [核心修改] 根据摇杆的两轴值，预先计算所有可能的速度分量
        # ========================================================
        # 前后推摇杆 (norm_y) 始终控制 vx
        base_vx = float(-norm_y * self.max_linear_speed * scale)
        
        # 左右推摇杆 (norm_x) 根据模式不同，可能控制 wz(角速度) 或 vy(横移速度)
        base_wz = float(-norm_x * self.max_angular_speed * scale)
        base_vy = float(-norm_x * self.max_linear_speed * scale) 

        # 初始化输出值
        vx = base_vx
        vy = 0.0
        wz = 0.0
        
        # 模式 0/1/2 对应您口语中的 模式1/模式2/模式3
        if current_mode == 0:
            # 模式 1 (mode=0): 正常模式，摇杆 X 轴控制旋转，Y轴控制前后
            vy = 0.0
            wz = base_wz
            
        elif current_mode == 1:
            # 模式 2 (mode=1): 平移模式，摇杆 X 轴重映射为控制左右横移 (vy)，禁止旋转
            vy = base_vy
            wz = 0.0 
            
        elif current_mode == 2:
            # 模式 3 (mode=2): 原地旋转模式，摇杆 X 轴控制旋转，禁止平移与前后移动
            vx = 0.0
            vy = 0.0
            wz = base_wz

        # 组装 Twist 消息并发布
        msg = Twist()
        msg.linear.x = float(vx)
        msg.linear.y = float(vy)
        msg.angular.z = float(wz)
        
        self.publisher_.publish(msg)

    def parse_zs_voltage(self, raw_val):
        """
        按照中盛手册解析电压：最高位为小数点位数，后四位为实际数值。
        """
        decimals = raw_val // 10000
        actual_val = raw_val % 10000
        voltage = actual_val / (10 ** decimals)
        return voltage

    def normalize_voltage(self, voltage):
        """
        将 0~5V 电压映射到 -1.0 ~ 1.0，并加入死区防止静止时漂移
        """
        center = 2.5
        max_v = 5.0
        min_v = 0.0
        deadzone = 0.15 # 2.35V ~ 2.65V 视为中心，防止误触和弹簧老化虚位
        
        # 限制在物理极值内
        voltage = max(min_v, min(voltage, max_v))
        
        if abs(voltage - center) < deadzone:
            return 0.0
            
        if voltage > center:
            norm = (voltage - center - deadzone) / (max_v - center - deadzone)
        else:
            norm = (voltage - center + deadzone) / (center - min_v - deadzone)
            
        return norm

    def check_crc16(self, data: bytes) -> bool:
        """
        标准的 Modbus RTU CRC16 校验算法
        """
        crc = 0xFFFF
        for pos in data[:-2]:
            crc ^= pos
            for _ in range(8):
                if (crc & 1) != 0:
                    crc >>= 1
                    crc ^= 0xA001
                else:
                    crc >>= 1
        # Modbus 传输时 CRC 低位在前，高位在后
        received_crc = data[-2] | (data[-1] << 8)
        return crc == received_crc


def main(args=None):
    rclpy.init(args=args)
    node = ModbusAnalogJoystick()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 安全退出：发送刹车并关闭串口
        node.publisher_.publish(Twist())
        if hasattr(node, 'serial_port') and node.serial_port.is_open:
            node.serial_port.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()