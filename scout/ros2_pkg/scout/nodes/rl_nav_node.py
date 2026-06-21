#!/usr/bin/env python3
"""Scout 局部导航节点, 使用 ONNX 策略推理和安全的降级控制."""
import math, os, sys, time
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "../../../../../shared"))

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool, Float32MultiArray, String
from qos_profiles import QOS_COMMAND

try:
    import onnxruntime as ort

    ORT_OK = True
except ImportError:
    ORT_OK = False


N_SEC = 36
MAX_RANGE = 6.0
OBS_DIM = 42


class ScoutRLNav(Node):
    def __init__(self):
        super().__init__("scout_rl_nav")
        self.declare_parameter("model_path", "/home/robot/models/scout_policy.onnx")
        self.declare_parameter("norm_path", "/home/robot/models/scout_norm.npz")
        self.declare_parameter("max_vx", 0.6)
        self.declare_parameter("max_wz", 2.0)
        self.declare_parameter("safety_dist", 0.30)
        self.declare_parameter("slow_dist", 0.60)
        self.declare_parameter("hz", 10.0)

        self._max_vx = float(self.get_parameter("max_vx").value)
        self._max_wz = float(self.get_parameter("max_wz").value)
        self._safety = float(self.get_parameter("safety_dist").value)
        self._slow = float(self.get_parameter("slow_dist").value)

        self._sess = None
        self._inp = None
        self._out = None
        self._load_model()
        self._load_norm()

        self._lidar = np.full(N_SEC, MAX_RANGE, np.float32)
        self._vx = 0.0
        self._wz = 0.0
        self._rx = 0.0
        self._ry = 0.0
        self._ryaw = 0.0
        self._goal_x = 5.0
        self._goal_y = 4.5
        self._goal_dist = 0.0
        self._goal_angle = 0.0
        self._life_conf = 0.0
        self._life_range = 5.0
        self._enabled = True
        self._scanning = False
        self._last_lidar = time.time()

        self.create_subscription(Float32MultiArray, "/scout/obstacle_distances", self._lidar_cb, 5)
        self.create_subscription(Odometry, "/scout/odom", self._odom_cb, 10)
        self.create_subscription(PoseStamped, "/scout/next_goal", self._goal_cb, 5)
        self.create_subscription(Float32MultiArray, "/scout/life_detections", self._life_cb, 10)
        self.create_subscription(Bool, "/scout/radar_trigger", self._scan_cb, 10)
        self.create_subscription(String, "/scout/rl_enable", self._enable_cb, 10)

        self._pub_cmd = self.create_publisher(Twist, "/scout/cmd_vel", QOS_COMMAND)
        self._pub_status = self.create_publisher(String, "/scout/rl_status", 10)
        self.create_timer(1.0 / float(self.get_parameter("hz").value), self._step)

    def _load_model(self):
        if not ORT_OK:
            self.get_logger().warn("onnxruntime not found; using fallback controller")
            return
        try:
            opts = ort.SessionOptions()
            opts.intra_op_num_threads = 4
            self._sess = ort.InferenceSession(
                self.get_parameter("model_path").value,
                opts,
                providers=["CPUExecutionProvider"],
            )
            self._inp = self._sess.get_inputs()[0].name
            self._out = self._sess.get_outputs()[0].name
        except Exception as exc:
            self.get_logger().warn(f"Model load failed; using fallback controller: {exc}")
            self._sess = None

    def _load_norm(self):
        try:
            data = np.load(self.get_parameter("norm_path").value)
            self._mean = data["mean"].astype(np.float32)
            self._var = data["var"].astype(np.float32)
        except Exception:
            self._mean = np.zeros(OBS_DIM, np.float32)
            self._var = np.ones(OBS_DIM, np.float32)

    def _lidar_cb(self, msg: Float32MultiArray):
        arr = np.asarray(msg.data, dtype=np.float32)
        if arr.size == N_SEC:
            self._lidar = np.clip(arr, 0.0, MAX_RANGE)
            self._last_lidar = time.time()

    def _odom_cb(self, msg: Odometry):
        self._rx = msg.pose.pose.position.x
        self._ry = msg.pose.pose.position.y
        self._vx = msg.twist.twist.linear.x
        self._wz = msg.twist.twist.angular.z
        q = msg.pose.pose.orientation
        self._ryaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self._update_goal_error()

    def _goal_cb(self, msg: PoseStamped):
        self._goal_x = msg.pose.position.x
        self._goal_y = msg.pose.position.y
        self._update_goal_error()

    def _scan_cb(self, msg: Bool):
        self._scanning = bool(msg.data)

    def _life_cb(self, msg: Float32MultiArray):
        if len(msg.data) >= 4:
            self._life_range = float(msg.data[0])
            self._life_conf = float(msg.data[3])

    def _enable_cb(self, msg: String):
        self._enabled = msg.data.strip().lower() == "true"

    def _update_goal_error(self):
        dx = self._goal_x - self._rx
        dy = self._goal_y - self._ry
        self._goal_dist = math.hypot(dx, dy)
        self._goal_angle = self._wrap(math.atan2(dy, dx) - self._ryaw)

    def _build_obs(self):
        obs = np.concatenate(
            [
                self._lidar / MAX_RANGE,
                [self._vx / self._max_vx, self._wz / self._max_wz],
                [np.clip(self._goal_dist / 15.0, 0.0, 1.0), self._goal_angle / math.pi],
                [self._life_conf, np.clip(self._life_range / 5.0, 0.0, 1.0)],
            ]
        ).astype(np.float32)
        return np.clip((obs - self._mean) / np.sqrt(self._var + 1e-8), -10.0, 10.0)

    def _step(self):
        cmd = Twist()
        min_d = float(np.min(self._lidar))

        if (
            self._scanning
            or not self._enabled
            or time.time() - self._last_lidar > 1.0
            or min_d < self._safety
            or self._goal_dist < 0.35
        ):
            self._pub_cmd.publish(cmd)
            self._publish_status(cmd, min_d)
            return

        action = self._policy_action()
        scale = np.clip((min_d - self._safety) / (self._slow - self._safety + 1e-6), 0.0, 1.0)
        cmd.linear.x = float(max(0.0, action[0])) * self._max_vx * float(scale)
        cmd.angular.z = float(np.clip(action[1], -1.0, 1.0)) * self._max_wz
        self._pub_cmd.publish(cmd)
        self._publish_status(cmd, min_d)

    def _policy_action(self):
        if self._sess:
            obs = self._build_obs()
            return self._sess.run([self._out], {self._inp: obs.reshape(1, -1)})[0][0]
        return np.asarray(
            [
                np.clip(self._goal_dist / 3.0, 0.0, 1.0),
                np.clip(self._goal_angle / math.pi, -1.0, 1.0),
            ],
            dtype=np.float32,
        )

    def _publish_status(self, cmd: Twist, min_d: float):
        msg = String()
        msg.data = (
            f"vx={cmd.linear.x:.2f} wz={cmd.angular.z:.2f} "
            f"min_d={min_d:.2f} goal_d={self._goal_dist:.2f}"
        )
        self._pub_status.publish(msg)

    @staticmethod
    def _wrap(angle):
        return math.atan2(math.sin(angle), math.cos(angle))


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(ScoutRLNav())
    rclpy.shutdown()
