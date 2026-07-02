import math
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose

class Nav2Commander:
    def __init__(self, node, success_callback, fail_callback):
        self.node = node
        self.logger = node.get_logger()
        self.success_callback = success_callback
        self.fail_callback = fail_callback
        
        self.nav_client = ActionClient(self.node, NavigateToPose, 'navigate_to_pose')

    def send_goal(self, coords):
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.logger.error("[Nav2] ❌ 服务器连接超时。")
            self.fail_callback()
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.node.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = coords[0]
        goal_msg.pose.pose.position.y = coords[1]
        goal_msg.pose.pose.orientation.z = math.sin(coords[2] / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(coords[2] / 2.0)
        
        self.send_goal_future = self.nav_client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.logger.warn('[Nav2] ⚠️ 目标点被规划器拒绝。')
            self.fail_callback()
            return
        
        self.logger.info('[Nav2] 🚀 目标已接受，路径追踪开始。')
        self.goal_result_future = goal_handle.get_result_async()
        self.goal_result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        status = future.result().status
        if status == 4: # SUCCEEDED
            self.logger.info('[Nav2] ✅ 全局导航阶段到达。')
            self.success_callback()
        else:
            self.logger.warn(f'[Nav2] ⚠️ 任务未能成功完成 (Status: {status})。')
            self.fail_callback()