#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped

class PointPicker(Node):
    def __init__(self):
        super().__init__('point_picker')
        self.subscription = self.create_subscription(
            PointStamped,
            '/clicked_point',
            self.listener_callback,
            10)
        self.point_count = 1
        self.get_logger().info('🟢 取点小助手已启动！请在 RViz 中使用 Publish Point 点击地图。')

    def listener_callback(self, msg):
        x = round(msg.point.x, 3)
        y = round(msg.point.y, 3)
        print(f"第 {self.point_count} 个点: [{x}, {y}],")
        self.point_count += 1

def main():
    rclpy.init()
    node = PointPicker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()