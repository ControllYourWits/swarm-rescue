#!/usr/bin/env python3
"""
ground_station_node.py -- 中枢指挥站节点.

订阅:
  /scout/hw_status  /scout/rl_status  /scout/life_detections
  /carrier/battery  /carrier/network_ok  /carrier/supply_status
  /specialist/hw_status  /specialist/arm_status  /specialist/thermal_status

发布:
  /swarm/status            汇总状态 JSON
  /scout/set_mode          模式/运动指令
  /carrier/set_mode
  /specialist/set_mode
  /carrier/supply_request  物资投送触发
  /specialist/arm_task     机械臂动作
  /specialist/led_cmd      LED 控制
  /gs/command_feedback     指令拒绝时的错误反馈
"""
import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Float32MultiArray, String


# ── 命令格式定义: (合法机器人, 合法动作, 参数个数) ─────────
COMMAND_SCHEMA = {
    "scout": {
        "stop":     0,
        "normal":   0,
        "rc":       0,
        "emergency": 0,
        "rl_enable": 1,   # arg: true|false
        "set_mode":  1,   # arg: stop|normal|rc|follow
        "supply":    2,   # args: item slot (routed to carrier)
    },
    "carrier": {
        "stop":     0,
        "follow":   0,
        "emergency": 0,
        "supply":   2,    # args: item slot  e.g. supply water 0
        "set_mode": 1,
    },
    "specialist": {
        "stop":      0,
        "emergency": 0,
        "arm_task":  1,   # arg: push|clear|deliver|home|inspect
        "led":       5,   # args: mode brt r g b
        "set_mode":  1,
    },
}


def parse_command(raw: str):
    """解析并校验地面站操作员指令.

    Returns (robot, action, args_list, error_message).
    成功时 error_message 为 None.
    """
    if not raw or not raw.strip():
        return None, None, [], "empty command"

    parts = [p.strip() for p in raw.strip().split(":")]
    if len(parts) < 2:
        return None, None, [], (
            f"invalid format '{raw}' -- expected robot:action[:args...]")

    robot = parts[0].lower()
    action = parts[1].lower()
    args = parts[2:]

    if robot not in COMMAND_SCHEMA:
        return None, None, [], (
            f"unknown robot '{robot}' -- "
            f"valid: {', '.join(sorted(COMMAND_SCHEMA))}")

    actions = COMMAND_SCHEMA[robot]
    if action not in actions:
        return None, None, [], (
            f"unknown action '{action}' for {robot} -- "
            f"valid: {', '.join(sorted(actions))}")

    expected_arity = actions[action]
    if len(args) != expected_arity:
        return None, None, [], (
            f"'{robot}:{action}' expects {expected_arity} argument(s), "
            f"got {len(args)}")

    return robot, action, args, None


class GroundStation(Node):
    LOW_BATTERY_PCT = 20.0
    AUTO_SUPPLY_CONF = 0.75
    AUTO_INSPECT_CONF = 0.65
    AUTO_LIGHT_DIST = 3.0
    CONF_RESET = 0.30

    def __init__(self):
        super().__init__("ground_station")

        self._state = {
            "scout": {
                "hw": "unknown", "rl": "unknown",
                "life_conf": 0.0, "life_range": 0.0,
            },
            "carrier": {
                "battery": 100.0, "network": True,
                "supply": "unknown",
            },
            "specialist": {
                "hw": "unknown", "arm": "IDLE",
                "thermal": "unknown",
            },
        }
        self._mission_log = []
        self._auto_supply_done = False
        self._auto_inspect_done = False

        # ── scout ──
        self.create_subscription(
            String, "/scout/hw_status",
            lambda m: self._update("scout", "hw", m.data), 10)
        self.create_subscription(
            String, "/scout/rl_status",
            lambda m: self._update("scout", "rl", m.data), 10)
        self.create_subscription(
            Float32MultiArray, "/scout/life_detections",
            self._life_cb, 10)

        # ── carrier ──
        self.create_subscription(
            Float32, "/carrier/battery",
            lambda m: self._update("carrier", "battery", m.data), 10)
        self.create_subscription(
            Bool, "/carrier/network_ok",
            lambda m: self._update("carrier", "network", m.data), 10)
        self.create_subscription(
            String, "/carrier/supply_status",
            lambda m: self._update("carrier", "supply", m.data), 10)

        # ── specialist ──
        self.create_subscription(
            String, "/specialist/hw_status",
            lambda m: self._update("specialist", "hw", m.data), 10)
        self.create_subscription(
            String, "/specialist/arm_status",
            lambda m: self._update("specialist", "arm", m.data), 10)
        self.create_subscription(
            String, "/specialist/thermal_status",
            lambda m: self._update("specialist", "thermal", m.data), 10)

        # ── publishers ──
        self._pub_status    = self.create_publisher(String, "/swarm/status",   5)
        self._pub_scout_m   = self.create_publisher(String, "/scout/set_mode", 10)
        self._pub_carrier_m = self.create_publisher(String, "/carrier/set_mode", 10)
        self._pub_spec_m    = self.create_publisher(String, "/specialist/set_mode", 10)
        self._pub_supply    = self.create_publisher(String, "/carrier/supply_request", 10)
        self._pub_arm       = self.create_publisher(String, "/specialist/arm_task", 10)
        self._pub_led       = self.create_publisher(String, "/specialist/led_cmd", 10)
        self._pub_feedback  = self.create_publisher(String, "/gs/command_feedback", 10)

        self.create_subscription(String, "/gs/command", self._cmd_cb, 10)
        self.create_timer(1.0, self._monitor)
        self.create_timer(2.0, self._publish_status)

        self.get_logger().info("Ground station online")

    def _update(self, robot, key, value):
        self._state[robot][key] = value

    def _life_cb(self, msg: Float32MultiArray):
        if len(msg.data) >= 4:
            self._state["scout"]["life_range"] = msg.data[0]
            self._state["scout"]["life_conf"] = msg.data[3]

    # ── 自动化监控 ─────────────────────────────────────────
    def _monitor(self):
        # low-battery auto-stop
        battery = self._state["carrier"]["battery"]
        if battery < self.LOW_BATTERY_PCT:
            self.get_logger().warn(f"Carrier low battery {battery:.0f}%")
            self._pub_carrier_m.publish(self._make_str("stop"))
            self._log(f"Carrier auto-stop (battery {battery:.0f}%)")

        # life-detection auto-supply
        conf = self._state["scout"]["life_conf"]
        if conf >= self.AUTO_SUPPLY_CONF and not self._auto_supply_done:
            self.get_logger().info(
                f"High-confidence life detection ({conf:.2f}), auto-deploy")
            self._pub_supply.publish(self._make_str("water:0"))
            self._pub_led.publish(self._make_str("1:200:255:255:200"))
            self._auto_supply_done = True
            self._log(f"Auto supply triggered (conf={conf:.2f})")

        if conf < self.CONF_RESET:
            self._auto_supply_done = False
            self._auto_inspect_done = False

        # life-detection auto-dispatch specialist
        if conf >= self.AUTO_INSPECT_CONF and not self._auto_inspect_done:
            arm_status = self._state["specialist"]["arm"]
            if arm_status != "BUSY":
                self.get_logger().info(
                    f"Life detected (conf={conf:.2f}), dispatching Specialist")
                self._pub_arm.publish(self._make_str("inspect"))
                self._auto_inspect_done = True
                self._log(f"Auto dispatch specialist:inspect (conf={conf:.2f})")

    def _publish_status(self):
        payload = json.dumps({
            "timestamp": self.get_clock().now().to_msg().sec,
            "scout": self._state["scout"],
            "carrier": self._state["carrier"],
            "specialist": self._state["specialist"],
            "log_count": len(self._mission_log),
        })
        self._pub_status.publish(self._make_str(payload))

    # ── 命令处理 ───────────────────────────────────────────
    def _cmd_cb(self, msg: String):
        raw = msg.data.strip()
        robot, action, args, error = parse_command(raw)

        if error:
            self._send_feedback(raw, error)
            return

        if robot == "scout":
            if action == "supply":
                # "supply:item:slot" alias from raw text
                self._pub_supply.publish(
                    self._make_str(f"{args[0]}:{args[1]}"))
            else:
                self._pub_scout_m.publish(self._make_str(action))

        elif robot == "carrier":
            if action == "supply":
                self._pub_supply.publish(
                    self._make_str(f"{args[0]}:{args[1]}"))
            else:
                self._pub_carrier_m.publish(self._make_str(action))

        elif robot == "specialist":
            if action == "arm_task":
                self._pub_arm.publish(self._make_str(args[0]))
            elif action == "led":
                self._pub_led.publish(
                    self._make_str(":".join(args)))
            else:
                self._pub_spec_m.publish(self._make_str(action))

        self._log(f"CMD [{robot}]: {action} {' '.join(args)}".rstrip())

    def _send_feedback(self, raw_cmd: str, error: str):
        self.get_logger().warn(f"Rejected command '{raw_cmd}': {error}")
        self._pub_feedback.publish(self._make_str(
            f"REJECTED '{raw_cmd}': {error}"))
        self._log(f"REJECTED '{raw_cmd}' -- {error}", level="warn")

    # ── helpers ────────────────────────────────────────────────
    @staticmethod
    def _make_str(data: str) -> String:
        s = String()
        s.data = data
        return s

    def _log(self, msg: str, level: str = "info"):
        entry = f"[{time.strftime('%H:%M:%S')}] {msg}"
        self._mission_log.append(entry)
        if level == "warn":
            self.get_logger().warn(msg)
        else:
            self.get_logger().info(entry)
        if len(self._mission_log) > 500:
            self._mission_log = self._mission_log[-200:]


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(GroundStation())
    rclpy.shutdown()
