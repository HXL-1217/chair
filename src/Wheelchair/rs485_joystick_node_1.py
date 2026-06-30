#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import time

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
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # 定时器频率 (20Hz = 50ms)。RS485 半双工问答需要时间，9600波特率下 20Hz 是比较稳定的极限
        self.timer = self.create_timer(0.05, self.timer_callback)
        
        # 速度限制
        self.max_linear_speed = 0.5   # m/s
        self.max_angular_speed = 1.0  # rad/s

        # 预先计算好的 Modbus 读取指令 (读取从站01的 0000H 和 0001H 两个寄存器，即CH1和CH2)
        # 01(站号) 04(功能码读输入) 00 00(起始地址) 00 02(读取2个寄存器) 71 CB(CRC校验)
        self.modbus_request = bytes([0x01, 0x04, 0x00, 0x00, 0x00, 0x02, 0x71, 0xCB])

    def timer_callback(self):
        try:
            # 1. 清空缓冲区，防止读取到上一轮的残余脏数据
            self.serial_port.reset_input_buffer()
            
            # 2. 发送读取请求
            self.serial_port.write(self.modbus_request)
            
            # 3. 读取响应 (期望读取 9 个字节: 01 04 04 CH1H CH1L CH2H CH2L CRCL CRCH)
            response = self.serial_port.read(9)
            
            if len(response) == 9:
                # 4. CRC16 校验，防止 RS485 线路干扰导致轮椅失控暴走
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
        
        # 解析中盛模块特有的“可变小数点”电压值
        voltage_y = self.parse_zs_voltage(ch1_raw) # 假设 CH1 接 Y轴 (前后)
        voltage_x = self.parse_zs_voltage(ch2_raw) # 假设 CH2 接 X轴 (左右)
        
        # 将 0~5V 电压归一化为 -1.0 ~ 1.0 (假设 2.5V 为摇杆中心静止位)
        norm_x = self.normalize_voltage(voltage_y)
        norm_y = self.normalize_voltage(voltage_x)
        
        # 组装 Twist 消息并发布
        msg = Twist()
        
        # 前推摇杆 (电压 > 2.5V)，轮椅前进
        msg.linear.x = float(-norm_y * self.max_linear_speed)
        
        # 右推摇杆 (电压 > 2.5V)，轮椅右转 (ROS中右转为负的角速度)
        msg.angular.z = float(-norm_x * self.max_angular_speed)
        
        self.publisher_.publish(msg)

    def parse_zs_voltage(self, raw_val):
        """
        按照中盛手册解析电压：最高位为小数点位数，后四位为实际数值。
        例: 31000 -> 3位小数, 数值1000 -> 1000 * 0.001 = 1.000 V
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