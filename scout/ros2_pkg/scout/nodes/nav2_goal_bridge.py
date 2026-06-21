#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
import math

try:
    from nav2_msgs.action import NavigateToPose
except ImportError:
    NavigateToPose = None

class Nav2GoalBridge(Node):
    def __init__(self):
        super().__init__('nav2_goal_bridge')

        if NavigateToPose is None:
            self.get_logger().fatal("nav2_msgs not installed; Nav2GoalBridge cannot start")
            raise RuntimeError("nav2_msgs not available")

        self.declare_parameter("dist_threshold", 0.5)
        self._dist_thresh = self.get_parameter("dist_threshold").value
        
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        self.create_subscription(PoseStamped, '/scout/next_goal', self._goal_cb, 5)
        
        self._current_goal = None
        self._goal_active = False
        self.get_logger().info("Nav2 Goal Bridge started. Waiting for /scout/next_goal...")

    def _goal_cb(self, msg: PoseStamped):
        # Check if the goal is significantly different from the current one
        if self._current_goal is not None:
            dx = msg.pose.position.x - self._current_goal.pose.position.x
            dy = msg.pose.position.y - self._current_goal.pose.position.y
            dist = math.hypot(dx, dy)
            if dist < self._dist_thresh and self._goal_active:
                return # Ignore small changes if currently tracking a goal
                
        self.get_logger().info(f"New goal received: x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f}")
        self._send_goal(msg)

    def _send_goal(self, pose_msg: PoseStamped):
        if not self._action_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn("NavigateToPose action server not available!")
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose_msg

        self.get_logger().info('Sending goal to Nav2...')
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self._goal_response_callback)
        
        self._current_goal = pose_msg
        self._goal_active = True

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected by Nav2')
            self._goal_active = False
            return

        self.get_logger().info('Goal accepted by Nav2, waiting for result...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self._get_result_callback)

    def _get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        self.get_logger().info(f'Nav2 Goal reached with status: {status}')
        self._goal_active = False

def main(args=None):
    rclpy.init(args=args)
    node = Nav2GoalBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
