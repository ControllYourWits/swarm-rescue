#!/usr/bin/env python3
"""Specialist arm task planner."""
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String


ACTIONS = {
    "home": [[90, 10, 10, 0]],
    "push": [[90, 60, 45, 0], [90, 70, 45, 0], [90, 60, 45, 0], [90, 10, 10, 0]],
    "clear": [
        [90, 45, 80, 1],
        [90, 50, 80, 0],
        [90, 20, 30, 0],
        [135, 20, 30, 0],
        [135, 10, 10, 1],
        [90, 10, 10, 0],
    ],
    "deliver": [[90, 60, 60, 0], [90, 70, 80, 0], [90, 70, 80, 1], [90, 10, 10, 0]],
    "inspect": [[45, 60, 45, 0], [90, 60, 45, 0], [135, 60, 45, 0], [90, 10, 10, 0]],
}


class ArmPlanner(Node):
    STEP_DELAY = 0.8

    def __init__(self):
        super().__init__("specialist_arm_planner")
        self._busy = False
        self._current = [90.0, 10.0, 10.0, 0.0]
        self.create_subscription(String, "/specialist/arm_task", self._task_cb, 10)
        self._pub = self.create_publisher(Float32MultiArray, "/specialist/arm_cmd", 10)
        self._pub_status = self.create_publisher(String, "/specialist/arm_status", 10)
        self._status("READY")

    def _task_cb(self, msg: String):
        if self._busy:
            self._status("BUSY")
            return
        key = msg.data.strip().lower().split(":")[0]
        if key not in ACTIONS:
            self.get_logger().warn(f"Unknown arm task: {msg.data}")
            self._status("UNKNOWN_TASK")
            return
        threading.Thread(target=self._execute, args=(key, ACTIONS[key]), daemon=True).start()

    def _execute(self, key: str, sequence: list):
        self._busy = True
        self._status(f"RUNNING:{key}")
        for step in sequence:
            out = Float32MultiArray()
            out.data = [float(value) for value in step]
            self._pub.publish(out)
            self._current = out.data
            time.sleep(self.STEP_DELAY)
        self._busy = False
        self._status("DONE")

    def _status(self, text: str):
        msg = String()
        msg.data = text
        self._pub_status.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(ArmPlanner())
    rclpy.shutdown()
