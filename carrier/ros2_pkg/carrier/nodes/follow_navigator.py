#!/usr/bin/env python3
"""Carrier follow controller.

Default mode publishes /carrier/cmd_vel directly, which keeps simulation and
basic hardware bringup independent of Nav2. Set controller_mode:=nav2 when a
Carrier Nav2 stack is launched and ready.
"""
import math

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Bool, String

try:
    from nav2_msgs.action import NavigateToPose
except ImportError:
    NavigateToPose = None


class FollowNavigator(Node):
    FOLLOW_DIST = 2.0
    MAX_VX = 0.6
    MAX_WZ = 1.5
    KP_DIST = 0.8
    KP_HEAD = 1.2
    STOP_DIST = 1.2

    def __init__(self):
        super().__init__("carrier_follow_nav")
        self.declare_parameter("follow_dist", self.FOLLOW_DIST)
        self.declare_parameter("namespace", "carrier")
        self.declare_parameter("target_topic", "/scout/odom")
        self.declare_parameter("controller_mode", "direct")

        self._follow_d = float(self.get_parameter("follow_dist").value)
        self._ns = str(self.get_parameter("namespace").value).strip("/")
        self._target_topic = str(self.get_parameter("target_topic").value)
        self._controller_mode = str(self.get_parameter("controller_mode").value).lower()

        self._tx = 0.0
        self._ty = 0.0
        self._tyaw = 0.0
        self._cx = 0.0
        self._cy = 0.0
        self._cyaw = 0.0
        self._target_ok = False
        self._self_ok = False
        self._scout_scanning = False
        self._enabled = True
        self._last_goal_x = None
        self._last_goal_y = None

        self.create_subscription(Odometry, self._target_topic, self._target_cb, 10)
        self.create_subscription(Odometry, f"/{self._ns}/odom", self._self_cb, 10)
        self.create_subscription(Bool, "/scout/radar_trigger", self._scan_cb, 10)
        self.create_subscription(String, f"/{self._ns}/nav_enable", self._enable_cb, 10)

        self._pub_cmd = self.create_publisher(Twist, f"/{self._ns}/cmd_vel", 10)
        self._pub_status = self.create_publisher(String, f"/{self._ns}/nav_status", 10)

        self._action_client = None
        if self._controller_mode == "nav2":
            if NavigateToPose is None:
                self.get_logger().warn("nav2_msgs unavailable; falling back to direct mode")
                self._controller_mode = "direct"
            else:
                self._action_client = ActionClient(self, NavigateToPose, f"/{self._ns}/navigate_to_pose")

        self.create_timer(0.1 if self._controller_mode == "direct" else 1.0, self._control)
        self.get_logger().info(
            f"Follow target={self._target_topic} mode={self._controller_mode} namespace={self._ns}"
        )

    def _enable_cb(self, msg: String):
        self._enabled = msg.data.strip().lower() == "true"

    def _target_cb(self, msg: Odometry):
        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y
        if self._target_ok:
            jump = math.hypot(px - self._tx, py - self._ty)
            if jump > 3.0:
                self.get_logger().warn(f"Target odom jump {jump:.1f}m ignored")
                return
        self._tx = px
        self._ty = py
        self._tyaw = self._yaw_from_odom(msg)
        self._target_ok = True

    def _self_cb(self, msg: Odometry):
        self._cx = msg.pose.pose.position.x
        self._cy = msg.pose.pose.position.y
        self._cyaw = self._yaw_from_odom(msg)
        self._self_ok = True

    def _scan_cb(self, msg: Bool):
        self._scout_scanning = bool(msg.data)

    def _control(self):
        if not self._enabled or not self._target_ok or self._scout_scanning:
            self._pub_cmd.publish(Twist())
            return

        target_x = self._tx - self._follow_d * math.cos(self._tyaw)
        target_y = self._ty - self._follow_d * math.sin(self._tyaw)

        if self._controller_mode == "nav2":
            self._send_nav2_goal(target_x, target_y)
        else:
            self._send_direct_cmd(target_x, target_y)

    def _send_direct_cmd(self, target_x: float, target_y: float):
        if not self._self_ok:
            self._pub_cmd.publish(Twist())
            return

        dx = target_x - self._cx
        dy = target_y - self._cy
        dist = math.hypot(dx, dy)
        head_err = self._wrap(math.atan2(dy, dx) - self._cyaw)

        cmd = Twist()
        if dist >= self.STOP_DIST:
            cmd.linear.x = min(self.KP_DIST * (dist - self.STOP_DIST), self.MAX_VX)
            cmd.angular.z = max(-self.MAX_WZ, min(self.MAX_WZ, self.KP_HEAD * head_err))
        self._pub_cmd.publish(cmd)

        status = String()
        status.data = f"dist={dist:.2f}m head_err={math.degrees(head_err):.1f}deg mode=direct"
        self._pub_status.publish(status)

    def _send_nav2_goal(self, target_x: float, target_y: float):
        if self._last_goal_x is not None:
            if math.hypot(target_x - self._last_goal_x, target_y - self._last_goal_y) < 0.5:
                return

        if not self._action_client.wait_for_server(timeout_sec=0.2):
            self.get_logger().warn(f"NavigateToPose action server not available for {self._ns}")
            return

        self._last_goal_x = target_x
        self._last_goal_y = target_y

        goal_msg = NavigateToPose.Goal()
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = "map"
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.pose.position.x = target_x
        pose_msg.pose.position.y = target_y
        q = self._quaternion_from_yaw(self._tyaw)
        pose_msg.pose.orientation.z = q[2]
        pose_msg.pose.orientation.w = q[3]
        goal_msg.pose = pose_msg
        self._action_client.send_goal_async(goal_msg)

        status = String()
        status.data = f"goal=({target_x:.2f},{target_y:.2f}) mode=nav2"
        self._pub_status.publish(status)

    @staticmethod
    def _yaw_from_odom(msg: Odometry):
        q = msg.pose.pose.orientation
        return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

    @staticmethod
    def _quaternion_from_yaw(yaw: float):
        return [0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)]

    @staticmethod
    def _wrap(angle: float):
        return math.atan2(math.sin(angle), math.cos(angle))


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(FollowNavigator())
    rclpy.shutdown()
