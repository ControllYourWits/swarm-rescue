#!/usr/bin/env python3
"""Life probability grid and next-goal publisher for Scout."""
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from std_msgs.msg import Bool, Float32MultiArray
from swarm_bringup.shared.qos_profiles import QOS_SENSOR


class LifeMapNode(Node):
    MAP_SIZE = 20.0
    RES = 0.10
    SPREAD = 0.5
    SCAN_INTERVAL = 2.0
    SCAN_WAIT = 3.5

    def __init__(self):
        super().__init__("life_map_node")
        self.declare_parameter("use_sim", False)
        self._use_sim = bool(self.get_parameter("use_sim").value)
        n = int(self.MAP_SIZE / self.RES)
        self._n = n
        self._life_map = np.zeros((n, n), np.float32)
        self._explored = np.zeros((n, n), np.bool_)
        self._rx = 0.0
        self._ry = 0.0
        self._ryaw = 0.0
        self._last_scan_x = -999.0
        self._last_scan_y = -999.0
        self._scanning = False
        self._scan_start = 0.0

        self.create_subscription(Float32MultiArray, "/scout/life_detections", self._life_cb, 10)
        self.create_subscription(Odometry, "/scout/odom", self._odom_cb, 10)
        self._pub_map = self.create_publisher(OccupancyGrid, "/scout/life_map", QOS_SENSOR)
        self._pub_trigger = self.create_publisher(Bool, "/scout/radar_trigger", 10)
        self._pub_goal = self.create_publisher(PoseStamped, "/scout/next_goal", 5)
        self.create_timer(0.5, self._update)

    def _odom_cb(self, msg: Odometry):
        self._rx = msg.pose.pose.position.x
        self._ry = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self._ryaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        cx, cy = self._world2cell(self._rx, self._ry)
        self._mark_explored(cx, cy, radius_m=0.4)

    def _life_cb(self, msg: Float32MultiArray):
        if len(msg.data) < 4:
            return
        dist, _breath, _heart, conf = [float(x) for x in msg.data[:4]]
        wx = self._rx + dist * math.cos(self._ryaw)
        wy = self._ry + dist * math.sin(self._ryaw)
        self._update_life_map(wx, wy, conf)

    def _update(self):
        if not self._use_sim:
            dist_from_last = math.hypot(self._rx - self._last_scan_x, self._ry - self._last_scan_y)
            if dist_from_last >= self.SCAN_INTERVAL and not self._scanning:
                self._trigger_scan()
            if self._scanning and (time.time() - self._scan_start) > self.SCAN_WAIT:
                self._scanning = False
                self._pub_trigger.publish(Bool(data=False))
        self._publish_map()
        self._publish_goal()

    def _trigger_scan(self):
        self._scanning = True
        self._scan_start = time.time()
        self._last_scan_x = self._rx
        self._last_scan_y = self._ry
        self._pub_trigger.publish(Bool(data=True))

    def _update_life_map(self, wx, wy, conf):
        cx, cy = self._world2cell(wx, wy)
        radius = int(self.SPREAD / self.RES) + 1
        sigma = max(self.SPREAD / self.RES, 1.0)
        for dx in range(-radius, radius + 1):
            for dy in range(-radius, radius + 1):
                nx = cx + dx
                ny = cy + dy
                if not self._in_bounds(nx, ny):
                    continue
                weight = conf * math.exp(-(dx * dx + dy * dy) / (2.0 * sigma * sigma))
                self._life_map[ny, nx] = min(1.0, self._life_map[ny, nx] + weight * 0.3)

    def _mark_explored(self, cx, cy, radius_m):
        radius = int(radius_m / self.RES)
        for dx in range(-radius, radius + 1):
            for dy in range(-radius, radius + 1):
                nx = cx + dx
                ny = cy + dy
                if self._in_bounds(nx, ny) and dx * dx + dy * dy <= radius * radius:
                    self._explored[ny, nx] = True

    def _publish_map(self):
        grid = OccupancyGrid()
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = "map"
        grid.info.resolution = self.RES
        grid.info.width = self._n
        grid.info.height = self._n
        grid.info.origin.position.x = -self.MAP_SIZE / 2.0
        grid.info.origin.position.y = -self.MAP_SIZE / 2.0
        grid.info.origin.orientation.w = 1.0
        grid.data = np.clip(self._life_map * 100.0, 0, 100).astype(np.int8).ravel().tolist()
        self._pub_map.publish(grid)

    def _publish_goal(self):
        if np.max(self._life_map) > 0.35:
            y, x = np.unravel_index(np.argmax(self._life_map), self._life_map.shape)
            gx, gy = self._cell2world(x, y)
        else:
            gx, gy = self._patrol_waypoints()
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = gx
        msg.pose.position.y = gy
        msg.pose.orientation.w = 1.0
        self._pub_goal.publish(msg)

    def _patrol_waypoints(self):
        # 无生命体征时的预定巡逻点（匹配15x15m仿真地图）
        offsets = [(4.5, 4.0), (4.5, -4.0), (-4.0, 4.0), (-4.0, -4.0), (0.0, 5.0)]
        return min(offsets, key=lambda p: math.hypot(p[0] - self._rx, p[1] - self._ry))

    def _world2cell(self, x, y):
        cx = int((x + self.MAP_SIZE / 2.0) / self.RES)
        cy = int((y + self.MAP_SIZE / 2.0) / self.RES)
        return cx, cy

    def _cell2world(self, cx, cy):
        x = cx * self.RES - self.MAP_SIZE / 2.0
        y = cy * self.RES - self.MAP_SIZE / 2.0
        return x, y

    def _in_bounds(self, cx, cy):
        return 0 <= cx < self._n and 0 <= cy < self._n


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(LifeMapNode())
    rclpy.shutdown()
