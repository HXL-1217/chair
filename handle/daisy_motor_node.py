import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import serial
import threading

class DaisyMotorNode(Node):
    def __init__(self):
        super().__init__('daisy_motor_node')
        
        # 串口配置 (请确保与你的udev规则一致)
        self.port = "/dev/ttyS2"
        self.baud = 115200
        self.timeout = 0.05  # 较短的超时时间，防止阻塞
        
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=self.timeout)
            self.get_logger().info(f"串联电机串口已连接: {self.port} @ {self.baud}")
            self.get_logger().info("已监听 1号 - 5号 辅机电机指令...")
        except Exception as e:
            self.get_logger().error(f"串口打开失败: {e}")
            raise e
            
        self.lock = threading.Lock()
        
        # 订阅 1-5 号电机的话题
        self.create_subscription(Int32, '/motor_1/cmd', lambda msg: self.motor_cb("1", msg.data), 10)
        self.create_subscription(Int32, '/motor_2/cmd', lambda msg: self.motor_cb("2", msg.data), 10)
        self.create_subscription(Int32, '/motor_3/cmd', lambda msg: self.motor_cb("3", msg.data), 10)
        self.create_subscription(Int32, '/motor_4/cmd', lambda msg: self.motor_cb("4", msg.data), 10)
        self.create_subscription(Int32, '/motor_5/cmd', lambda msg: self.motor_cb("5", msg.data), 10) 

    def clamp(self, v: int, lo: int, hi: int) -> int:
        return max(lo, min(v, hi))

    def motor_cb(self, dev_id: str, percent: int):
        """收到 ROS2 话题消息后，转化为 ASCII 串口指令发送"""
        pct = self.clamp(percent, -100, 100)
        cmd = f"@{dev_id}.D:{pct};\r\n"
        
        with self.lock:
            try:
                self.ser.reset_input_buffer()
                self.ser.write(cmd.encode("ascii", errors="ignore"))
                self.ser.flush()
                
                # 快速读取应答 (调试时可取消注释)
                # resp = self.ser.readline().decode("ascii", errors="ignore").strip()
            except Exception as e:
                self.get_logger().error(f"串口通信错误: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = DaisyMotorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Int32
# import serial
# import threading

# class DaisyMotorNode(Node):
#     def __init__(self):
#         super().__init__('daisy_motor_node')
        
#         # 串口配置 (请确保与你的udev规则一致)
#         self.port = "/dev/ttyS2"
#         self.baud = 115200
#         self.timeout = 0.05  # 较短的超时时间，防止阻塞
        
#         try:
#             self.ser = serial.Serial(self.port, self.baud, timeout=self.timeout)
#             self.get_logger().info(f"串联电机串口已连接: {self.port} @ {self.baud}")
#             self.get_logger().info("已监听 1号 - 5号 辅机电机指令...")
#         except Exception as e:
#             self.get_logger().error(f"串口打开失败: {e}")
#             raise e
            
#         self.lock = threading.Lock()
        
#         # [新增] 用于记录上一次操作的电机编号
#         self.last_active_motor = None 
        
#         # 订阅 1-5 号电机的话题
#         self.create_subscription(Int32, '/motor_1/cmd', lambda msg: self.motor_cb("1", msg.data), 10)
#         self.create_subscription(Int32, '/motor_2/cmd', lambda msg: self.motor_cb("2", msg.data), 10)
#         self.create_subscription(Int32, '/motor_3/cmd', lambda msg: self.motor_cb("3", msg.data), 10)
#         self.create_subscription(Int32, '/motor_4/cmd', lambda msg: self.motor_cb("4", msg.data), 10)
#         self.create_subscription(Int32, '/motor_5/cmd', lambda msg: self.motor_cb("5", msg.data), 10) 

#     def clamp(self, v: int, lo: int, hi: int) -> int:
#         return max(lo, min(v, hi))

#     def motor_cb(self, dev_id: str, percent: int):
#         """收到 ROS2 话题消息后，转化为 ASCII 串口指令发送"""
        
#         # [新增] 判断是否切换了电机
#         if self.last_active_motor != dev_id:
#             if self.last_active_motor is not None:
#                 self.get_logger().info(f"🔄 切换电机: 从 [{self.last_active_motor}号] 切换至 [{dev_id}号]")
#             else:
#                 self.get_logger().info(f"▶️ 首次激活电机: [{dev_id}号]")
            
#             # 更新当前激活的电机编号
#             self.last_active_motor = dev_id

#         pct = self.clamp(percent, -100, 100)
#         cmd = f"@{dev_id}.D:{pct};\r\n"
        
#         with self.lock:
#             try:
#                 self.ser.reset_input_buffer()
#                 self.ser.write(cmd.encode("ascii", errors="ignore"))
#                 self.ser.flush()
                
#                 # 快速读取应答 (调试时可取消注释)
#                 # resp = self.ser.readline().decode("ascii", errors="ignore").strip()
#             except Exception as e:
#                 self.get_logger().error(f"串口通信错误: {e}")

# def main(args=None):
#     rclpy.init(args=args)
#     node = DaisyMotorNode()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.ser.close()
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()