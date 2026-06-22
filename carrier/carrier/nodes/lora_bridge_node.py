#!/usr/bin/env python3
"""Low-bandwidth LoRa backup bridge.

LoRa cannot carry DDS traffic. This node only forwards compact emergency
commands and low-rate status frames when the normal Carrier network is down.
"""
import math
import struct
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String

try:
    import serial
except ImportError:
    serial = None

LORA_HEADER = 0xAA
LORA_TAIL = 0x55

ROBOT_ID = {"scout": 0x01, "carrier": 0x02, "specialist": 0x03}

CMD_HEARTBEAT = 0x00
CMD_ESTOP = 0x01
CMD_RETURN_HOME = 0x02

STAT_MOVING = 0x01
STAT_SCANNING = 0x02
STAT_LIFE_FOUND = 0x04
STAT_EMERGENCY = 0x80

UPLINK_BODY_FMT = "<BBBBff"
UPLINK_BODY_LEN = struct.calcsize(UPLINK_BODY_FMT)
UPLINK_FRAME_LEN = 1 + UPLINK_BODY_LEN + 1 + 1
DOWNLINK_BODY_LEN = 3
DOWNLINK_FRAME_LEN = 1 + DOWNLINK_BODY_LEN + 1 + 1


def crc8(data: bytes) -> int:
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if (crc & 0x80) else (crc << 1) & 0xFF
    return crc


def pack_lora_uplink(robot_id: int, status: int, battery_pct: int, pos_x: float, pos_y: float) -> bytes:
    body = struct.pack(UPLINK_BODY_FMT, robot_id, status, battery_pct, 0, pos_x, pos_y)
    return bytes([LORA_HEADER]) + body + bytes([crc8(body), LORA_TAIL])


def unpack_lora_uplink(frame: bytes):
    if len(frame) != UPLINK_FRAME_LEN or frame[0] != LORA_HEADER or frame[-1] != LORA_TAIL:
        return None
    body = frame[1 : 1 + UPLINK_BODY_LEN]
    if crc8(body) != frame[-2]:
        return None
    rid, status, batt, _, px, py = struct.unpack(UPLINK_BODY_FMT, body)
    return {"robot_id": rid, "status": status, "battery_pct": batt, "pos_x": px, "pos_y": py}


def pack_lora_downlink(cmd_id: int, param: int = 0) -> bytes:
    body = bytes([cmd_id & 0xFF, param & 0xFF, 0x00])
    return bytes([LORA_HEADER]) + body + bytes([crc8(body), LORA_TAIL])


def unpack_lora_downlink(frame: bytes):
    if len(frame) != DOWNLINK_FRAME_LEN or frame[0] != LORA_HEADER or frame[-1] != LORA_TAIL:
        return None
    body = frame[1 : 1 + DOWNLINK_BODY_LEN]
    if crc8(body) != frame[-2]:
        return None
    return {"cmd_id": body[0], "param": body[1]}


class LoraBridge(Node):
    def __init__(self):
        super().__init__("carrier_lora_bridge")
        self.declare_parameter("port", "/dev/ttyUSB2")
        self.declare_parameter("baud", 115200)
        self.declare_parameter("robot_id", "carrier")
        self.declare_parameter("slot_offset", 0.33)

        self._rid = ROBOT_ID.get(str(self.get_parameter("robot_id").value), ROBOT_ID["carrier"])
        self._slot = float(self.get_parameter("slot_offset").value)
        self._network_ok = True
        self._lora_active = False
        self._ser = self._open_serial()

        self._odom_x = 0.0
        self._odom_y = 0.0
        self._battery_pct = 100.0
        self._status = 0
        self._teammates = {}

        self._pub_cmd = self.create_publisher(Twist, "/lora/cmd_vel", 10)
        self._pub_estop = self.create_publisher(Bool, "/lora/estop", 10)
        self._pub_teammate = self.create_publisher(String, "/lora/teammate_positions", 10)

        self.create_subscription(Bool, "/carrier/network_ok", self._net_cb, 10)
        self.create_subscription(Odometry, "/carrier/odom", self._odom_cb, 10)
        self.create_subscription(Float32, "/carrier/battery", self._batt_cb, 10)
        self.create_timer(2.0, self._check_link)

        self.get_logger().info(
            f"LoRa bridge robot_id={self.get_parameter('robot_id').value} slot_offset={self._slot}s"
        )

    def _open_serial(self):
        if serial is None:
            self.get_logger().warn("pyserial not installed; LoRa disabled")
            return None
        try:
            return serial.Serial(
                self.get_parameter("port").value,
                int(self.get_parameter("baud").value),
                timeout=0.05,
            )
        except Exception as exc:
            self.get_logger().warn(f"LoRa serial unavailable: {exc}")
            return None

    def _net_cb(self, msg: Bool):
        self._network_ok = bool(msg.data)

    def _odom_cb(self, msg: Odometry):
        self._odom_x = msg.pose.pose.position.x
        self._odom_y = msg.pose.pose.position.y
        self._status = STAT_MOVING if abs(msg.twist.twist.linear.x) > 0.02 else 0

    def _batt_cb(self, msg: Float32):
        self._battery_pct = float(msg.data)

    def _check_link(self):
        if self._network_ok:
            if self._lora_active:
                self.get_logger().info("Network restored; deactivating LoRa mode")
                self._lora_active = False
            return

        if not self._lora_active:
            self.get_logger().warn("Network lost; activating LoRa backup link")
            self._lora_active = True
            threading.Thread(target=self._lora_loop, daemon=True).start()

    def _lora_loop(self):
        while self._lora_active and rclpy.ok():
            cycle_start = time.time()
            target_time = math.ceil(cycle_start) + self._slot
            sleep_s = max(0.0, target_time - time.time())
            if sleep_s:
                time.sleep(sleep_s)

            if not self._lora_active:
                break

            self._send_uplink()
            listen_until = min(math.ceil(time.time()) - time.time(), 0.35)
            if listen_until > 0:
                self._listen_downlink(listen_until)
            self._listen_teammates()
            self._publish_teammates()
            remaining = math.ceil(time.time()) - time.time()
            if remaining > 0:
                time.sleep(remaining)

    def _send_uplink(self):
        if self._ser is None:
            return
        frame = pack_lora_uplink(
            self._rid,
            self._status,
            int(min(100.0, max(0.0, self._battery_pct))),
            self._odom_x,
            self._odom_y,
        )
        try:
            self._ser.write(frame)
        except Exception as exc:
            self.get_logger().error(f"LoRa TX error: {exc}")

    def _listen_downlink(self, timeout_s: float):
        if self._ser is None:
            return
        deadline = time.time() + timeout_s
        buf = bytearray()
        while time.time() < deadline:
            try:
                chunk = self._ser.read(1)
            except Exception:
                break
            if not chunk:
                continue
            byte = chunk[0]
            if not buf and byte != LORA_HEADER:
                continue
            buf.append(byte)
            if len(buf) == DOWNLINK_FRAME_LEN:
                self._dispatch_downlink(bytes(buf))
                buf.clear()

    def _dispatch_downlink(self, frame: bytes):
        data = unpack_lora_downlink(frame)
        if data is None:
            return
        cmd_id = data["cmd_id"]

        if cmd_id == CMD_ESTOP:
            self.get_logger().error("LoRa E-STOP received")
            self._status |= STAT_EMERGENCY
            self._pub_estop.publish(Bool(data=True))
            self._pub_cmd.publish(Twist())
        elif cmd_id == CMD_RETURN_HOME:
            self.get_logger().warn("LoRa RETURN_HOME received")
            cmd = Twist()
            cmd.linear.x = -0.3
            self._pub_cmd.publish(cmd)

    def _listen_teammates(self):
        if self._ser is None:
            return
        try:
            while self._ser.in_waiting >= UPLINK_FRAME_LEN:
                frame = self._ser.read(UPLINK_FRAME_LEN)
                data = unpack_lora_uplink(frame)
                if data and data["robot_id"] != self._rid:
                    self._teammates[data["robot_id"]] = data
        except Exception:
            return

    def _publish_teammates(self):
        if not self._teammates:
            return
        items = [
            f"{rid}:x={data['pos_x']:.2f},y={data['pos_y']:.2f},bat={data['battery_pct']}"
            for rid, data in sorted(self._teammates.items())
        ]
        self._pub_teammate.publish(String(data=";".join(items)))


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(LoraBridge())
    rclpy.shutdown()
