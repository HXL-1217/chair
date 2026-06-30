import os
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
import threading

# 强制 pygame 使用虚拟视频驱动
os.environ["SDL_VIDEODRIVER"] = "dummy"
import pygame

class DevicesHandle:
    def __init__(self, joystick=None):
        pygame.init()
        pygame.joystick.init()

        if joystick is not None:
            self.joystick = joystick
        else:
            if pygame.joystick.get_count() == 0:
                raise Exception("未检测到手柄，请检查 USB 或蓝牙连接！")
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()

        self.done = False
        self.uAxes = [0] * 6
        self.uKey = [0] * 12
        self.uHat = [(0, 0)]
        self.lock = threading.Lock()

    def update(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.done = True

        with self.lock:
            for i in range(self.joystick.get_numaxes()):
                temp = self.joystick.get_axis(i)
                self.uAxes[i] = temp if abs(temp) > 0.05 else 0

            self.uKey = [self.joystick.get_button(i) for i in range(self.joystick.get_numbuttons())]
            self.uHat = [self.joystick.get_hat(i) for i in range(self.joystick.get_numhats())]

    def get_state(self):
        with self.lock:
            return self.uAxes.copy(), self.uKey.copy(), self.uHat.copy()


class CmdVelPublisherNode(Node):
    def __init__(self):
        super().__init__('joy_teleop_node')
        
        # 定义所有的发布者 (增加 m5)
        self.pub_track = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_m1 = self.create_publisher(Int32, '/motor_1/cmd', 10)
        self.pub_m2 = self.create_publisher(Int32, '/motor_2/cmd', 10)
        self.pub_m3 = self.create_publisher(Int32, '/motor_3/cmd', 10)
        self.pub_m4 = self.create_publisher(Int32, '/motor_4/cmd', 10)
        self.pub_m5 = self.create_publisher(Int32, '/motor_5/cmd', 10) # 【新增 5 号电机】
        
        # 频率与加速度参数
        self.dt = 0.02  # 控制频率 50Hz
        self.timer = self.create_timer(self.dt, self.timer_callback) 
        
        self.accel_v = 0.2   
        self.accel_w = 0.5   
        self.accel_m = 80.0  
        
        self.mod = 0  
        
        # 当前状态 (辅机电机数组扩充到 5 个)
        self.cur_v = 0.0  
        self.cur_w = 0.0  
        self.cur_m = [0.0, 0.0, 0.0, 0.0, 0.0] 

        try:
            self.joystick_handle = DevicesHandle()
            self.get_logger().info("手柄就绪！ 已启动")
            self.get_logger().info("按键映射: A->底盘, B->电机1, X->电机2, Y->电机3, L1->电机4, R1->电机5")
        except Exception as e:
            self.get_logger().error(f"手柄初始化失败: {e}")
            raise e

    def ramp_value(self, current, target, step):
        if target > current:
            return min(target, current + step)
        elif target < current:
            return max(target, current - step)
        else:
            return current

    def timer_callback(self):
        self.joystick_handle.update()
        axes, keys, hats = self.joystick_handle.get_state()

        # ======== 模式切换逻辑 ========
        new_mod = self.mod
        if keys[0] == 1: new_mod = 0     # A键  -> 底盘
        elif keys[1] == 1: new_mod = 1   # B键  -> 电机1
        elif keys[2] == 1: new_mod = 2   # X键  -> 电机2
        elif keys[3] == 1: new_mod = 3   # Y键  -> 电机3
        elif keys[4] == 1: new_mod = 4   # L1键 -> 电机4
        elif keys[5] == 1: new_mod = 5   # R1键 -> 电机5 

        if new_mod != self.mod:
            self.mod = new_mod
            modes = ["履带底盘", "1号辅机电机", "2号辅机电机", "3号辅机电机", "4号辅机电机", "5号辅机电机"]
            self.get_logger().info(f" 已切换至控制: {modes[self.mod]}")

        # ======== 1. 计算“目标期望值” ========
        target_v = 0.0
        target_w = 0.0
        target_m = [0.0, 0.0, 0.0, 0.0, 0.0]

        if self.mod == 0:
            target_v = float(-axes[1] * 0.5)  
            target_w = float(-axes[3] * 0.5)  
        else:
            idx = self.mod - 1
            target_m[idx] = float(-axes[1] * 100) 

        # ======== 2. 计算本次循环允许最大变化量 ========
        step_v = self.accel_v * self.dt
        step_w = self.accel_w * self.dt
        step_m = self.accel_m * self.dt

        # ======== 3. 执行平滑加速算法 ========
        self.cur_v = self.ramp_value(self.cur_v, target_v, step_v)
        self.cur_w = self.ramp_value(self.cur_w, target_w, step_w)
        for i in range(5):  # 循环更新 5 个电机
            self.cur_m[i] = self.ramp_value(self.cur_m[i], target_m[i], step_m)

        # ======== 4. 统一下发指令 ========
        msg = Twist()
        msg.linear.x = self.cur_v
        msg.angular.z = self.cur_w
        self.pub_track.publish(msg)
        
        self.pub_m1.publish(Int32(data=int(self.cur_m[0])))
        self.pub_m2.publish(Int32(data=int(self.cur_m[1])))
        self.pub_m3.publish(Int32(data=int(self.cur_m[2])))
        self.pub_m4.publish(Int32(data=int(self.cur_m[3])))
        self.pub_m5.publish(Int32(data=int(self.cur_m[4]))) # 【新增发布 5 号电机话题】

def main(args=None):
    rclpy.init(args=args)
    try:
        node = CmdVelPublisherNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()