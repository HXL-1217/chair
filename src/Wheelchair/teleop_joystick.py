#!/usr/bin/env python3
"""
teleop_joystick.py

ROS2 node (Python3) that reads joystick state from the existing `uHandle.DevicesHandle`
interface used in `Wheelchair.py` and publishes `geometry_msgs/msg/Twist` on `/cmd_vel`.

Usage:
  - Make executable: `chmod +x teleop_joystick.py`
  - Run directly after sourcing your workspace: `python3 teleop_joystick.py`
  - Or install into your package and use `ros2 run` after adding entry in CMakeLists/package.xml.

This node mirrors the axes mapping from the original `Wheelchair.py`:
  linear.x  = -axes[1] * linear_scale
  linear.y  = -axes[0] * linear_scale
  angular.z = -axes[2] * angular_scale

Buttons (keys) are read to set a `mod` that changes the speed multiplier.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import uHandle
import time


class TeleopJoystick(Node):
    def __init__(self):
        super().__init__('teleop_joystick')

        # Parameters
        self.declare_parameter('publish_rate_hz', 20)
        self.declare_parameter('linear_scale', 0.5)
        self.declare_parameter('angular_scale', 1.0)
        self.declare_parameter('deadzone', 0.05)

        self.rate_hz = self.get_parameter('publish_rate_hz').value
        self.linear_scale = self.get_parameter('linear_scale').value
        self.angular_scale = self.get_parameter('angular_scale').value
        self.deadzone = self.get_parameter('deadzone').value

        # speed multipliers for different modes (mod 0..4)
        self.mod_multipliers = [0.5, 1.0, 0.2, 0.8, 0.3]
        self.mod = 0

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # joystick handle
        self.device_handle = uHandle.DevicesHandle()

        # Timer
        timer_period = 1.0 / max(1.0, float(self.rate_hz))
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info('teleop_joystick node started (publishing /cmd_vel at %s Hz)' % self.rate_hz)

    def apply_deadzone(self, v: float) -> float:
        if abs(v) < self.deadzone:
            return 0.0
        return v

    def timer_callback(self):
        try:
            # update and read state
            self.device_handle.update()
            axes, keys, hats = self.device_handle.get_state()

            # safe defaults
            if axes is None:
                axes = [0.0, 0.0, 0.0]
            # ensure at least 3 axes exist
            while len(axes) < 3:
                axes.append(0.0)
            if keys is None:
                keys = [0]*6
            # hats can be empty; we don't strictly need it here

            # update mode from buttons (mirror Wheelchair.py mapping)
            if len(keys) >= 5:
                if keys[0] == 1:
                    self.mod = 0
                elif keys[1] == 1:
                    self.mod = 1
                elif keys[2] == 1:
                    self.mod = 2
                elif keys[3] == 1:
                    self.mod = 3
                elif keys[4] == 1:
                    self.mod = 4

            multiplier = 1.0
            if 0 <= self.mod < len(self.mod_multipliers):
                multiplier = self.mod_multipliers[self.mod]

            # mapping follows Wheelchair.py: [-axes[1]*0.5, -axes[0]*0.5, -axes[2]]
            lx = -self.apply_deadzone(axes[1]) * self.linear_scale * multiplier
            ly = -self.apply_deadzone(axes[0]) * self.linear_scale * multiplier
            az = -self.apply_deadzone(axes[2]) * self.angular_scale * multiplier

            twist = Twist()
            twist.linear.x = float(lx)
            twist.linear.y = float(ly)
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = float(az)

            self.pub.publish(twist)

        except Exception as e:
            # Log but keep running
            self.get_logger().error('Exception in teleop timer_callback: %s' % str(e))

    def destroy_node(self):
        # make sure to stop handle
        try:
            self.device_handle.stop()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TeleopJoystick()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt, shutting down')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
