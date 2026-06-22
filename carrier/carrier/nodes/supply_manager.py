#!/usr/bin/env python3
"""Carrier supply deployment manager."""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String


SUPPLY_SLOTS = {
    "water": 0,
    "firstaid": 1,
    "battery": 2,
    "rope": 3,
}


class SupplyManager(Node):
    AUTO_SUPPLY_CONF = 0.7

    def __init__(self):
        super().__init__("carrier_supply_manager")
        self._inventory = {name: True for name in SUPPLY_SLOTS}
        self._auto_supply = False

        self.create_subscription(String, "/carrier/supply_request", self._request_cb, 10)
        self.create_subscription(Float32MultiArray, "/scout/life_detections", self._life_cb, 10)
        self.create_subscription(String, "/carrier/auto_supply", self._auto_cb, 10)

        self._pub_cmd = self.create_publisher(String, "/carrier/supply_cmd", 10)
        self._pub_status = self.create_publisher(String, "/carrier/supply_status", 10)
        self._publish_status("ready")

    def _auto_cb(self, msg: String):
        self._auto_supply = msg.data.strip().lower() == "true"
        self._publish_status("auto_supply=" + str(self._auto_supply).lower())

    def _request_cb(self, msg: String):
        item = msg.data.split(":")[0].strip().lower()
        if item not in SUPPLY_SLOTS:
            self.get_logger().warn(f"Unknown supply item: {item}")
            return
        if not self._inventory[item]:
            self.get_logger().warn(f"{item} out of stock")
            self._publish_status(f"{item} out_of_stock")
            return
        self._deploy(item)

    def _life_cb(self, msg: Float32MultiArray):
        if not self._auto_supply or len(msg.data) < 4:
            return
        if float(msg.data[3]) < self.AUTO_SUPPLY_CONF:
            return
        for item in ("water", "firstaid"):
            if self._inventory.get(item, False):
                self._deploy(item)
                break

    def _deploy(self, item: str):
        slot = SUPPLY_SLOTS[item]
        cmd = String()
        cmd.data = f"throw:{slot}"
        self._pub_cmd.publish(cmd)
        self._inventory[item] = False
        self._publish_status(f"deployed {item} slot={slot}")

    def _publish_status(self, prefix: str):
        msg = String()
        available = ",".join([k for k, v in self._inventory.items() if v])
        msg.data = f"{prefix}; available={available or 'none'}"
        self._pub_status.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(SupplyManager())
    rclpy.shutdown()
