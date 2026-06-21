#!/usr/bin/env python3
"""Lightweight three-robot simulator support node.

This is not a physics replacement for Gazebo. It publishes deterministic odom,
laser sectors, and periodic life-sign detections so Scout, Carrier, Specialist,
map merge, and the dashboard can be exercised together without real hardware.
Gazebo still provides the visual world and spawned robot entities.
"""
import math
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32MultiArray, String


@dataclass
class RobotState:
    x: float
    y: float
    yaw: float
    vx: float = 0.0
    wz: float = 0.0


class SimSwarmNode(Node):
    DT = 0.05
    MAX_RANGE = 6.0
    N_SECTORS = 36

    def __init__(self):
        super().__init__("sim_swarm_node")
        self.declare_parameter("life_scan_period", 8.0)

        self._robots = {
            "scout": RobotState(0.0, 0.0, 0.0),
            "carrier": RobotState(-1.8, 0.0, 0.0),
            "specialist": RobotState(-3.2, -0.2, 0.0),
        }
        self._cmd = {name: Twist() for name in self._robots}
        self._debris = [
            (2.0, 1.5, 0.45),
            (-1.5, 2.5, 0.45),
            (3.5, -2.0, 0.55),
            (-3.0, -1.5, 0.40),
            (0.5, 4.0, 0.45),
            (-4.0, 3.0, 0.55),
            (4.5, 3.5, 0.40),
            (-2.5, -4.0, 0.50),
        ]
        self._survivors = [(2.5, 3.5), (-3.5, -3.0)]
        self._scan_active = False

        for name in self._robots:
            self.create_subscription(
                Twist, f"/{name}/cmd_vel", self._make_cmd_cb(name), 10
            )

        self._odom_pubs = {
            name: self.create_publisher(Odometry, f"/{name}/odom", 10)
            for name in self._robots
        }
        self._scan_pub = self.create_publisher(LaserScan, "/scout/scan", 10)
        self._sector_pub = self.create_publisher(
            Float32MultiArray, "/scout/obstacle_distances", 10
        )
        self._goal_pub = self.create_publisher(PoseStamped, "/scout/next_goal", 10)
        self._life_pub = self.create_publisher(
            Float32MultiArray, "/scout/life_detections", 10
        )
        self._radar_debug_pub = self.create_publisher(String, "/scout/radar_debug", 10)
        self._radar_trigger_pub = self.create_publisher(Bool, "/scout/radar_trigger", 10)
        self._status_pub = self.create_publisher(String, "/swarm/status", 10)

        self.create_timer(self.DT, self._tick)
        self.create_timer(0.5, self._publish_goal)
        self.create_timer(1.0, self._publish_status)
        self.create_timer(float(self.get_parameter("life_scan_period").value), self._scan_life)

    def _make_cmd_cb(self, name):
        def cb(msg):
            self._cmd[name] = msg

        return cb

    def _tick(self):
        for name, state in self._robots.items():
            cmd = self._cmd[name]
            state.vx = max(-0.6, min(0.6, float(cmd.linear.x)))
            state.wz = max(-2.0, min(2.0, float(cmd.angular.z)))
            state.yaw = self._wrap(state.yaw + state.wz * self.DT)
            state.x += state.vx * math.cos(state.yaw) * self.DT
            state.y += state.vx * math.sin(state.yaw) * self.DT
            state.x = max(-7.2, min(7.2, state.x))
            state.y = max(-7.2, min(7.2, state.y))
            self._publish_odom(name, state)

        sectors = self._sector_distances(self._robots["scout"])
        self._sector_pub.publish(Float32MultiArray(data=sectors))
        self._publish_scan(sectors)

    def _publish_odom(self, name, state):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.child_frame_id = f"{name}/base_link"
        msg.pose.pose.position.x = state.x
        msg.pose.pose.position.y = state.y
        msg.pose.pose.orientation.z = math.sin(state.yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(state.yaw / 2.0)
        msg.twist.twist.linear.x = state.vx
        msg.twist.twist.angular.z = state.wz
        self._odom_pubs[name].publish(msg)

    def _publish_scan(self, sectors):
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = "scout/laser_link"
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = (scan.angle_max - scan.angle_min) / len(sectors)
        scan.range_min = 0.05
        scan.range_max = self.MAX_RANGE
        scan.ranges = sectors
        self._scan_pub.publish(scan)

    def _publish_goal(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = 5.0
        msg.pose.position.y = 4.5
        msg.pose.orientation.w = 1.0
        self._goal_pub.publish(msg)

    def _publish_status(self):
        scout = self._robots["scout"]
        carrier = self._robots["carrier"]
        specialist = self._robots["specialist"]
        msg = String()
        msg.data = (
            '{"scout":{"online":true,"x":%.2f,"y":%.2f},'
            '"carrier":{"online":true,"x":%.2f,"y":%.2f},'
            '"specialist":{"online":true,"x":%.2f,"y":%.2f}}'
            % (scout.x, scout.y, carrier.x, carrier.y, specialist.x, specialist.y)
        )
        self._status_pub.publish(msg)

    def _scan_life(self):
        self._radar_trigger_pub.publish(Bool(data=True))
        scout = self._robots["scout"]
        best = None
        for sx, sy in self._survivors:
            dx = sx - scout.x
            dy = sy - scout.y
            dist = math.hypot(dx, dy)
            if dist > 5.0:
                continue
            bearing = self._wrap(math.atan2(dy, dx) - scout.yaw)
            confidence = max(0.0, 1.0 - dist / 5.0) * max(0.0, 1.0 - abs(bearing) / 1.7)
            if best is None or confidence > best[3]:
                best = (dist, 0.25, 1.2, confidence)

        if best and best[3] > 0.25:
            self._life_pub.publish(Float32MultiArray(data=list(best)))
            debug = String()
            debug.data = (
                "range=%.2fm breath=0.25Hz(SNR15.0dB) "
                "heart=1.20Hz(SNR12.0dB) life_conf=%.2f" % (best[0], best[3])
            )
            self._radar_debug_pub.publish(debug)
        self._radar_trigger_pub.publish(Bool(data=False))

    def _sector_distances(self, state):
        values = []
        for i in range(self.N_SECTORS):
            angle = state.yaw + (2.0 * math.pi * i / self.N_SECTORS)
            values.append(self._ray_distance(state.x, state.y, angle))
        return values

    def _ray_distance(self, x, y, angle):
        dx = math.cos(angle)
        dy = math.sin(angle)
        best = self.MAX_RANGE
        for ox, oy, radius in self._debris:
            fx = ox - x
            fy = oy - y
            b = 2.0 * (fx * dx + fy * dy)
            c = fx * fx + fy * fy - radius * radius
            disc = b * b - 4.0 * c
            if disc >= 0.0:
                t = (-b - math.sqrt(disc)) / 2.0
                if 0.05 < t < best:
                    best = t
        return float(best)

    @staticmethod
    def _wrap(angle):
        return math.atan2(math.sin(angle), math.cos(angle))


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(SimSwarmNode())
    rclpy.shutdown()
