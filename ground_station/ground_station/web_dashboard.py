#!/usr/bin/env python3
"""Web dashboard for swarm rescue ground station.

Flask + flask-socketio real-time dashboard that bridges ROS2 topics to browser.
Run: ros2 run ground_station web_dashboard
"""
import json
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Float32MultiArray, String

try:
    from flask import Flask, render_template, send_from_directory
    from flask_socketio import SocketIO
    FLASK_OK = True
except ImportError:
    FLASK_OK = False

app = Flask(__name__,
            template_folder="templates",
            static_folder="static")
socketio = SocketIO(app, cors_allowed_origins="*", async_mode="threading")

# Shared state updated by ROS callbacks
_state = {
    "scout":    {"online": False, "x": 0.0, "y": 0.0, "mode": "unknown", "rl": "unknown"},
    "carrier":  {"online": False, "x": 0.0, "y": 0.0, "battery": 100.0, "network": True, "supply": "unknown"},
    "specialist": {"online": False, "x": 0.0, "y": 0.0, "arm": "IDLE", "thermal": "unknown"},
    "life": {"confidence": 0.0, "range": 0.0},
    "log": [],
}
_lock = threading.Lock()


@app.route("/")
def index():
    return render_template("dashboard.html")


@app.route("/api/status")
def api_status():
    with _lock:
        return json.dumps(_state)


def push_state():
    """Push current state to all WebSocket clients."""
    with _lock:
        socketio.emit("state_update", _state)


class WebDashboardNode(Node):
    def __init__(self):
        super().__init__("web_dashboard")

        # Scout
        self.create_subscription(String, "/scout/hw_status",
                                 lambda m: self._update("scout", "hw", m.data), 10)
        self.create_subscription(String, "/scout/rl_status",
                                 lambda m: self._update("scout", "rl", m.data), 10)

        # Carrier
        self.create_subscription(Float32, "/carrier/battery",
                                 lambda m: self._update("carrier", "battery", m.data), 10)
        self.create_subscription(Bool, "/carrier/network_ok",
                                 lambda m: self._update("carrier", "network", m.data), 10)
        self.create_subscription(String, "/carrier/supply_status",
                                 lambda m: self._update("carrier", "supply", m.data), 10)
        self.create_subscription(String, "/carrier/hw_status",
                                 lambda m: self._update("carrier", "hw", m.data), 10)

        # Specialist
        self.create_subscription(String, "/specialist/hw_status",
                                 lambda m: self._update("specialist", "hw", m.data), 10)
        self.create_subscription(String, "/specialist/arm_status",
                                 lambda m: self._update("specialist", "arm", m.data), 10)
        self.create_subscription(String, "/specialist/thermal_status",
                                 lambda m: self._update("specialist", "thermal", m.data), 10)

        # Life detections
        self.create_subscription(Float32MultiArray, "/scout/life_detections",
                                 self._life_cb, 10)

        # Swarm status (JSON from ground station)
        self.create_subscription(String, "/swarm/status", self._swarm_cb, 10)

        # Command input from web
        self._pub_cmd = self.create_publisher(String, "/gs/command", 10)

        self.create_timer(1.0, self._push)
        self.get_logger().info("Web dashboard started")

    def _update(self, robot, key, value):
        with _lock:
            if robot in _state:
                _state[robot][key] = value
                if key == "hw":
                    _state[robot]["online"] = value != "OFFLINE"

    def _life_cb(self, msg):
        if len(msg.data) >= 4:
            with _lock:
                _state["life"]["range"] = float(msg.data[0])
                _state["life"]["confidence"] = float(msg.data[3])

    def _swarm_cb(self, msg):
        try:
            data = json.loads(msg.data)
            with _lock:
                for robot in ("scout", "carrier", "specialist"):
                    if robot in data and isinstance(data[robot], dict):
                        _state[robot].update(
                            {k: v for k, v in data[robot].items()
                             if k in ("x", "y", "online")})
        except (json.JSONDecodeError, KeyError):
            pass

    def _push(self):
        push_state()

    def send_command(self, cmd: str):
        msg = String()
        msg.data = cmd
        self._pub_cmd.publish(msg)
        with _lock:
            _state["log"].append(cmd)
            if len(_state["log"]) > 100:
                _state["log"] = _state["log"][-50:]


# Global node reference for Flask routes
_node = None


@socketio.on("command")
def handle_command(data):
    """Handle commands from web client."""
    cmd = data.get("cmd", "")
    if _node and cmd:
        _node.send_command(cmd)
        _node.get_logger().info(f"Web cmd: {cmd}")


def main(args=None):
    global _node
    if not FLASK_OK:
        print("ERROR: flask and flask-socketio are required. "
              "Install with: pip install flask flask-socketio")
        return

    rclpy.init(args=args)
    _node = WebDashboardNode()

    # Run Flask in a separate thread
    flask_thread = threading.Thread(
        target=lambda: socketio.run(app, host="0.0.0.0", port=5000,
                                     allow_unsafe_werkzeug=True),
        daemon=True,
    )
    flask_thread.start()

    try:
        rclpy.spin(_node)
    except KeyboardInterrupt:
        pass
    finally:
        _node.destroy_node()
        rclpy.shutdown()
