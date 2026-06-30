import os
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math

# 【关键修改】强制 pygame 使用虚拟视频驱动，防止在无显示器的香橙派上报错
os.environ["SDL_VIDEODRIVER"] = "dummy"
import pygame
import time
import threading

class DevicesHandle:
    def __init__(self, joystick=None):
        pygame.init()
        pygame.joystick.init()

        if joystick is not None:
            self.joystick = joystick
        else:
            joystick_count = pygame.joystick.get_count()
            if joystick_count == 0:
                raise Exception("未检测到手柄，请检查 USB 或蓝牙连接！")
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()

        self.done = False
        self.uAxes = [0, 0, -1, 0, 0, -1]
        self.uKey = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]  # A, B, X, Y, L, R, l, r, 0, LA, RA, 0
        self.uHat = [(0, 0)]
        self.lock = threading.Lock()

    def update(self):
        """更新手柄状态"""
        for event in pygame.event.get():  # 获取所有事件
            if event.type == pygame.QUIT:  # 如果用户点击关闭
                self.done = True

        with self.lock:
            # 更新轴状态
            for i in range(self.joystick.get_numaxes()):
                temp = self.joystick.get_axis(i)
                self.uAxes[i] = temp if abs(temp) > 0.04 else 0

            # 更新按键状态
            self.uKey = [self.joystick.get_button(i) for i in range(self.joystick.get_numbuttons())]

            # 更新方向键状态
            self.uHat = [self.joystick.get_hat(i) for i in range(self.joystick.get_numhats())]

    def get_state(self):
        """获取当前手柄状态"""
        with self.lock:
            return self.uAxes.copy(), self.uKey.copy(), self.uHat.copy()

    def stop(self):
        """停止更新"""
        self.done = True

class CmdVelPublisherNode(Node):
    def __init__(self):
        super().__init__('cmd_vel_publisher_node')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.t = 0.0
        self.mod = 0
        
        try:
            self.joystick_handle = DevicesHandle()
            self.get_logger().info("Cmd Vel Publisher Node Started (Orange Pi Headless Mode)")
        except Exception as e:
            self.get_logger().error(f"手柄初始化失败: {e}")
            raise e

    def timer_callback(self):
        self.joystick_handle.update()  # 在主线程中更新手柄状态
        axes, keys, hats = self.joystick_handle.get_state()  # 获取当前状态
        
        speed = [-axes[1]*0.5, -axes[0]*0.5, -axes[3]]
        
        if keys[0] == 1:
            self.mod = 0
        elif keys[1] == 1:
            self.mod = 1
        elif keys[2] == 1:
            self.mod = 2
        elif keys[3] == 1:
            self.mod = 3
            
        self.t += 0.1

        if self.mod == 0:
            speed[1] = 0
        elif self.mod == 1:
            speed[2] = 0
        elif self.mod == 2:
            speed[0] = 0
            speed[1] = 0
        # elif self.mod == 3:
        #     speed[1] = 0

        msg = Twist()
        msg.linear.x = float(speed[0])
        msg.linear.y = float(speed[1])
        msg.angular.z = float(speed[2])
        # self.get_logger().info(f"{msg.linear.x:.2f}")
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    try:
        node = CmdVelPublisherNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass  # 忽略 Ctrl+C 产生的报错堆栈
    finally:
        # 优雅地销毁节点和关闭 rclpy
        if 'node' in locals():
            node.joystick_handle.stop()
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()