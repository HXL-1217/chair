#!/usr/bin/env python3
import threading
import time
import serial

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Int32MultiArray


class VoiceProfileSerialNode(Node):
    """
    串口接收语音模块的“模式/速度”设置，发布到 /drive_profile:
      msg.data = [mode, speed_level]
    mode: 0/1/2
    speed_level: 0/1/2
    """

    def __init__(self):
        super().__init__('voice_profile_serial_node')

        # ---- parameters ----
        self.declare_parameter('port', '/dev/ttyUSB2')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('protocol', 'byte')  # 'byte' or 'line'
        # byte 协议下：用这两组“单字节码”映射 mode/speed_level
        self.declare_parameter('mode_codes', [0x11, 0x12, 0x13])
        self.declare_parameter('speed_codes', [0x21, 0x22, 0x23])
        self.declare_parameter('publish_topic', '/drive_profile')
        self.declare_parameter('publish_hz', 2.0)  # 周期性发布，避免丢状态

        self.port = self.get_parameter('port').value
        self.baud = int(self.get_parameter('baud').value)
        self.protocol = self.get_parameter('protocol').value
        self.mode_codes = [int(x) for x in self.get_parameter('mode_codes').value]
        self.speed_codes = [int(x) for x in self.get_parameter('speed_codes').value]
        self.topic = self.get_parameter('publish_topic').value
        self.publish_hz = float(self.get_parameter('publish_hz').value)

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.pub = self.create_publisher(Int32MultiArray, self.topic, qos)

        # ---- state ----
        self._lock = threading.Lock()
        self.mode = 0
        self.speed_level = 1  # 默认中速
        self._changed = True

        # ---- serial ----
        self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
        self.get_logger().info(
            f"VoiceProfileSerialNode started. port={self.port}, baud={self.baud}, protocol={self.protocol}"
        )

        self._running = True
        self._th = threading.Thread(target=self._read_loop, daemon=True)
        self._th.start()

        # 周期性发布（也会在变化时尽快发布）
        period = 1.0 / self.publish_hz if self.publish_hz > 0 else 0.5
        self._timer = self.create_timer(period, self._publish_timer)

    def _publish_timer(self):
        # 变化了就发；即使没变化也周期发一次（保证下游稳定拿到）
        with self._lock:
            mode = self.mode
            speed = self.speed_level
            changed = self._changed
            self._changed = False

        msg = Int32MultiArray()
        msg.data = [int(mode), int(speed)]
        self.pub.publish(msg)

        if changed:
            self.get_logger().info(f"Updated profile: mode={mode}, speed_level={speed}")

    def _read_loop(self):
        while self._running and rclpy.ok():
            try:
                if self.protocol == 'line':
                    line = self.ser.readline()
                    if not line:
                        continue
                    self._handle_line(line)
                else:
                    b = self.ser.read(1)
                    if not b:
                        continue
                    self._handle_byte(b[0])
            except Exception as e:
                self.get_logger().error(f"Serial error: {e}")
                time.sleep(0.2)

    def _handle_byte(self, val: int):
        # 单字节协议：val 属于某个列表，则更新 mode 或 speed_level
        if val in self.mode_codes:
            new_mode = self.mode_codes.index(val)
            with self._lock:
                if new_mode != self.mode:
                    self.mode = new_mode
                    self._changed = True
            return

        if val in self.speed_codes:
            new_speed = self.speed_codes.index(val)
            with self._lock:
                if new_speed != self.speed_level:
                    self.speed_level = new_speed
                    self._changed = True
            return

    def _handle_line(self, line: bytes):
        """
        行协议可支持两种简单格式（二选一即可）：
          1) "M1 S2\n" 或 "M1S2\n"
          2) "1,2\n"  （mode,speed）
        """
        s = line.decode('utf-8', errors='ignore').strip().replace(' ', '')
        if not s:
            return

        # 1) MxSy
        if 'M' in s and 'S' in s:
            try:
                m = int(s.split('M', 1)[1].split('S', 1)[0])
                v = int(s.split('S', 1)[1])
                m = max(0, min(2, m))
                v = max(0, min(2, v))
                with self._lock:
                    if m != self.mode or v != self.speed_level:
                        self.mode = m
                        self.speed_level = v
                        self._changed = True
                return
            except:
                return

        # 2) "mode,speed"
        if ',' in s:
            try:
                parts = s.split(',')
                m = int(parts[0]); v = int(parts[1])
                m = max(0, min(2, m))
                v = max(0, min(2, v))
                with self._lock:
                    if m != self.mode or v != self.speed_level:
                        self.mode = m
                        self.speed_level = v
                        self._changed = True
            except:
                return

    def destroy_node(self):
        self._running = False
        try:
            if self.ser:
                self.ser.close()
        except:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VoiceProfileSerialNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
