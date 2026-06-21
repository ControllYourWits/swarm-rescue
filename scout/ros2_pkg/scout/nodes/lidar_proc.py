#!/usr/bin/env python3
"""Convert Scout LaserScan data into fixed sector obstacle distances."""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, Float32MultiArray


class LidarProc(Node):
    def __init__(self):
        super().__init__("scout_lidar_proc")
        self.declare_parameter("n_sectors", 36)
        self.declare_parameter("max_range", 6.0)
        self.declare_parameter("min_range", 0.08)
        self.declare_parameter("use_sim", False)
        self._n = int(self.get_parameter("n_sectors").value)
        self._max = float(self.get_parameter("max_range").value)
        self._min = float(self.get_parameter("min_range").value)
        self._use_sim = bool(self.get_parameter("use_sim").value)

        if self._use_sim:
            self.get_logger().info("LidarProc in sim mode — not subscribing to /scout/scan")
            return

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=5,
        )
        self.create_subscription(LaserScan, "/scout/scan", self._cb, qos)
        self._pub_obs = self.create_publisher(Float32MultiArray, "/scout/obstacle_distances", 10)
        self._pub_near = self.create_publisher(Float32, "/scout/nearest_obstacle", 10)

    def _cb(self, scan: LaserScan):
        raw = np.asarray(scan.ranges, dtype=np.float32)
        if raw.size == 0:
            return
        raw = np.where(np.isfinite(raw), raw, self._max)
        raw = np.clip(raw, self._min, self._max)

        filt = self._median_filter(raw, width=3)
        sectors = self._to_sectors(filt)

        msg = Float32MultiArray()
        msg.data = sectors.tolist()
        self._pub_obs.publish(msg)

        nearest = Float32()
        nearest.data = float(np.min(filt))
        self._pub_near.publish(nearest)

    def _median_filter(self, values, width):
        half = width // 2
        return np.asarray(
            [np.median(values[max(0, i - half) : i + half + 1]) for i in range(len(values))],
            dtype=np.float32,
        )

    def _to_sectors(self, values):
        chunks = np.array_split(values, self._n)
        return np.asarray(
            [float(np.min(chunk)) if len(chunk) else self._max for chunk in chunks],
            dtype=np.float32,
        )


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(LidarProc())
    rclpy.shutdown()
