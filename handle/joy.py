#!/usr/bin/env python3
import os
import time
import threading

# 【关键修改】强制 pygame 使用虚拟视频驱动，防止在无显示器的香橙派上报错
os.environ["SDL_VIDEODRIVER"] = "dummy"
import pygame

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray


class DevicesHandle:
    """
    pygame 读取手柄状态：axes/buttons/hats
    """
    def __init__(self, joystick_index: int = 0, deadzone: float = 0.04):
        pygame.init()
        pygame.joystick.init()

        self.deadzone = float(deadzone)

        joystick_count = pygame.joystick.get_count()
        if joystick_count == 0:
            raise RuntimeError("未检测到手柄（pygame.joystick.get_count()==0），请检查 USB 或蓝牙连接！")

        if joystick_index >= joystick_count:
            raise RuntimeError(f"手柄索引 {joystick_index} 超出范围，当前数量={joystick_count}")

        self.joystick = pygame.joystick.Joystick(joystick_index)
        self.joystick.init()

        self.done = False
        self.lock = threading.Lock()

        # 预分配（不同手柄轴数量不同，按常见 6 轴）
        self.uAxes = [0.0] * max(6, self.joystick.get_numaxes())
        self.uKey = [0] * max(12, self.joystick.get_numbuttons())
        self.uHat = [(0, 0)] * max(1, self.joystick.get_numhats())

        self.last_update_time = time.time()

    def update(self):
        """更新手柄状态（在定时器线程调用即可）"""
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.done = True

        with self.lock:
            # axes
            naxes = self.joystick.get_numaxes()
            if len(self.uAxes) < naxes:
                self.uAxes = [0.0] * naxes
            for i in range(naxes):
                v = float(self.joystick.get_axis(i))
                self.uAxes[i] = v if abs(v) > self.deadzone else 0.0  # 过滤死区

            # buttons
            nbtn = self.joystick.get_numbuttons()
            if len(self.uKey) < nbtn:
                self.uKey = [0] * nbtn
            self.uKey = [self.joystick.get_button(i) for i in range(nbtn)]

            # hats
            nhat = self.joystick.get_numhats()
            if nhat <= 0:
                self.uHat = [(0, 0)]
            else:
                self.uHat = [self.joystick.get_hat(i) for i in range(nhat)]

            self.last_update_time = time.time()

    def get_state(self):
        with self.lock:
            return self.uAxes.copy(), self.uKey.copy(), self.uHat.copy(), self.last_update_time

    def stop(self):
        self.done = True


class JoyCmdVelWithProfile(Node):
    """
    - 订阅 /drive_profile: Int32MultiArray [mode, speed_level]
    - 发布 /cmd_vel: Twist
    - 手柄只管摇杆，模式/速度由语音设置
    """
    def __init__(self):
        super().__init__('joy_cmdvel_with_profile')

        # ---------------- Parameters ----------------
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('profile_topic', '/drive_profile')

        self.declare_parameter('publish_hz', 10.0)

        # 手柄设置
        self.declare_parameter('joystick_index', 0)
        self.declare_parameter('deadzone', 0.04)

        # 轴映射（默认：axes[1]=左摇杆Y 前后，axes[0]=左摇杆X 横移，axes[3]=右摇杆X 旋转）
        self.declare_parameter('axis_vx', 1)  # 前后：-axes[1]
        self.declare_parameter('axis_vy', 0)  # 左右：-axes[0]
        self.declare_parameter('axis_wz', 3)  # 旋转：-axes[3]

        # 最大速度（未乘档位倍率前）
        self.declare_parameter('max_vx', 0.5)   # m/s
        self.declare_parameter('max_vy', 0.5)   # m/s
        self.declare_parameter('max_wz', 1.0)   # rad/s

        # 三档速度倍率
        self.declare_parameter('speed_scale', [0.4, 0.7, 1.0])

        # deadman：超过这个时间没更新手柄就停
        self.declare_parameter('deadman_sec', 0.5)

        # ---------------- Read Parameters ----------------
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.profile_topic = self.get_parameter('profile_topic').value
        self.publish_hz = float(self.get_parameter('publish_hz').value)

        self.joystick_index = int(self.get_parameter('joystick_index').value)
        self.deadzone = float(self.get_parameter('deadzone').value)

        self.axis_vx = int(self.get_parameter('axis_vx').value)
        self.axis_vy = int(self.get_parameter('axis_vy').value)
        self.axis_wz = int(self.get_parameter('axis_wz').value)

        self.max_vx = float(self.get_parameter('max_vx').value)
        self.max_vy = float(self.get_parameter('max_vy').value)
        self.max_wz = float(self.get_parameter('max_wz').value)

        self.speed_scale = [float(x) for x in self.get_parameter('speed_scale').value]
        if len(self.speed_scale) != 3:
            self.get_logger().warn("speed_scale 长度不是 3，自动回退为 [0.4, 0.7, 1.0]")
            self.speed_scale = [0.4, 0.7, 1.0]

        self.deadman_sec = float(self.get_parameter('deadman_sec').value)

        # ---------------- ROS pubs/subs ----------------
        self.pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.profile_sub = self.create_subscription(
            Int32MultiArray,
            self.profile_topic,
            self.profile_callback,
            10
        )

        # 当前配置：默认 mode=0 speed_level=1（中速）
        self.mode = 0
        self.speed_level = 1
        self._profile_lock = threading.Lock()

        # ---------------- Joystick ----------------
        try:
            self.joy = DevicesHandle(joystick_index=self.joystick_index, deadzone=self.deadzone)
        except Exception as e:
            self.get_logger().error(str(e))
            # 没手柄也能启动，但只会发布 0
            self.joy = None

        # timer
        period = 1.0 / self.publish_hz if self.publish_hz > 0 else 0.1
        self.timer = self.create_timer(period, self.timer_cb)

        self.get_logger().info(
            f"Started (Orange Pi Headless). cmd_vel={self.cmd_vel_topic}, profile={self.profile_topic}, "
            f"axis(vx,vy,wz)=({self.axis_vx},{self.axis_vy},{self.axis_wz}), "
            f"max(vx,vy,wz)=({self.max_vx},{self.max_vy},{self.max_wz})"
        )

    def profile_callback(self, msg: Int32MultiArray):
        if len(msg.data) < 2:
            return
        m = int(msg.data[0])
        s = int(msg.data[1])
        m = max(0, min(2, m))
        s = max(0, min(2, s))
        with self._profile_lock:
            changed = (m != self.mode) or (s != self.speed_level)
            self.mode = m
            self.speed_level = s
        if changed:
            self.get_logger().info(f"Profile updated: mode={m}, speed_level={s}")

    def _get_profile(self):
        with self._profile_lock:
            return self.mode, self.speed_level

    def timer_cb(self):
        # 默认停
        out = Twist()

        if self.joy is None:
            self.pub.publish(out)
            return

        # 更新手柄状态
        try:
            self.joy.update()
            axes, keys, hats, t_last = self.joy.get_state()
        except Exception as e:
            self.get_logger().warn(f"Joystick read failed: {e}")
            self.pub.publish(out)
            return

        # deadman：太久没更新就停
        if (time.time() - t_last) > self.deadman_sec:
            self.pub.publish(out)
            return

        # 读模式/档位
        mode, speed_level = self._get_profile()
        k = self.speed_scale[speed_level]

        # 取轴值（越界保护）
        def axis(idx: int) -> float:
            if idx < 0 or idx >= len(axes):
                return 0.0
            return float(axes[idx])

        # 按你原程序：速度 = [-axes[1]*0.5, -axes[0]*0.5, -axes[3]]
        vx = -axis(self.axis_vx) * self.max_vx * k
        vy = -axis(self.axis_vy) * self.max_vy * k
        wz = -axis(self.axis_wz) * self.max_wz * k

        # 三种运动模式（沿用你之前 mod 的语义）
        # mode=0: 禁止横移（只前后 + 转）
        # mode=1: 禁止旋转（只平移）
        # mode=2: 只允许旋转（禁平移）
        if mode == 0:
            vy = 0.0
        elif mode == 1:
            wz = 0.0  # 禁止旋转
        elif mode == 2:
            vx = 0.0
            vy = 0.0

        out.linear.x = float(vx)
        out.linear.y = float(vy)
        out.angular.z = float(wz)

        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = JoyCmdVelWithProfile()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass  # 忽略 Ctrl+C 产生的长篇报错信息，实现优雅退出
    finally:
        if node.joy is not None:
            node.joy.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()