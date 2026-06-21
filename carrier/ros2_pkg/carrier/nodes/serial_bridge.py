#!/usr/bin/env python3
"""serial_bridge.py — Carrier 串口桥
发布 /carrier/odom  /carrier/battery
订阅 /carrier/cmd_vel  /carrier/supply_cmd
"""
import math, threading, sys, os, time
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "../../../../../shared/protocol"))
import rclpy
from rclpy.node import Node
import serial
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Float32
from swarm_protocol import (FrameParser, MsgUp, ChassisMode,
                             make_cmd_vel, make_set_mode, make_heartbeat, make_supply_cmd)
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "../../../../../shared"))
from qos_profiles import QOS_SENSOR
from robot_state import RobotStateMachine, RobotState

class CarrierBridge(Node):
    def __init__(self):
        super().__init__("carrier_serial_bridge")
        self.declare_parameter("port", "/dev/ttyS3")
        self.declare_parameter("baud", 921600)
        port = self.get_parameter("port").value
        baud = self.get_parameter("baud").value
        self._ser    = serial.Serial(port, baud, timeout=0.005)
        self._parser = FrameParser()
        self._last_cmd = self.get_clock().now()
        self._last_rx = time.time()
        self._stm32_online = False

        self._pub_odom = self.create_publisher(Odometry, "/carrier/odom",    QOS_SENSOR)
        self._pub_bat  = self.create_publisher(Float32,  "/carrier/battery", 10)
        self._pub_stat = self.create_publisher(String,   "/carrier/hw_status",10)

        self.create_subscription(Twist,  "/carrier/cmd_vel",    self._cmd_cb,    10)
        self.create_subscription(String, "/carrier/supply_cmd", self._supply_cb, 10)
        self.create_subscription(String, "/carrier/set_mode",   self._mode_cb,   10)

        self.create_timer(1.0, lambda: self._ser.write(make_heartbeat()))
        self.create_timer(0.1, self._timeout_cb)
        self.create_timer(2.0, self._watchdog_cb)
        self._state_machine = RobotStateMachine("carrier", self)
        threading.Thread(target=self._rx, daemon=True).start()
        self.get_logger().info(f"Carrier bridge {port}@{baud}")

    def _cmd_cb(self, msg):
        self._ser.write(make_cmd_vel(msg.linear.x, msg.linear.y, msg.angular.z))
        self._last_cmd = self.get_clock().now()

    def _supply_cb(self, msg):
        # format: "open:0" / "close:1" / "throw:2"
        parts = msg.data.split(":")
        actions = {"open":1,"close":0,"throw":2}
        action = actions.get(parts[0].lower(), 0)
        slot   = int(parts[1]) if len(parts)>1 else 0
        self._ser.write(make_supply_cmd(action, slot))

    def _mode_cb(self, msg):
        m = {"stop":ChassisMode.STOP,"normal":ChassisMode.NORMAL,
             "follow":ChassisMode.FOLLOW,"emergency":ChassisMode.EMERGENCY}
        self._ser.write(make_set_mode(m.get(msg.data.lower(), ChassisMode.STOP)))

    def _timeout_cb(self):
        elapsed = (self.get_clock().now()-self._last_cmd).nanoseconds*1e-9
        if elapsed > 0.8:
            self._ser.write(make_set_mode(ChassisMode.STOP))

    def _watchdog_cb(self):
        elapsed = time.time() - self._last_rx
        was_online = self._stm32_online
        self._stm32_online = elapsed < 3.0
        if was_online and not self._stm32_online:
            self.get_logger().error("Carrier STM32 OFFLINE — no data for 3s")
            self._state_machine.emergency("stm32_offline")
            s = String(); s.data = "OFFLINE"
            self._pub_stat.publish(s)
        elif not was_online and self._stm32_online:
            self.get_logger().info("Carrier STM32 recovered")
            self._state_machine.recover()

    def _rx(self):
        while rclpy.ok():
            try:
                data = self._ser.read(64)
                if data:
                    self._last_rx = time.time()
                    for b in data:
                        r = self._parser.feed(b)
                        if r: self._dispatch(*r)
            except serial.SerialException as e:
                self.get_logger().error(f"Serial RX error: {e}, reconnecting...")
                self._reconnect_serial()
            except Exception as e:
                self.get_logger().error(f"RX parse error: {e}, continuing")
                time.sleep(0.05)

    def _reconnect_serial(self):
        self._ser.close()
        while rclpy.ok():
            try:
                time.sleep(1.0)
                self._ser.open()
                self._ser.reset_input_buffer()
                self.get_logger().info("Serial reconnected")
                return
            except Exception as e:
                self.get_logger().warn(f"Reconnect failed: {e}, retrying...")

    def _dispatch(self, mid, payload):
        now = self.get_clock().now().to_msg()
        if mid == MsgUp.ODOM:
            d = self._parser.parse_odom(payload)
            if not d: return
            msg = Odometry(); msg.header.stamp = now
            msg.header.frame_id = "odom"
            msg.child_frame_id  = "carrier/base_link"
            msg.pose.pose.position.x = d["pos_x"]
            msg.pose.pose.position.y = d["pos_y"]
            yaw = d["yaw"]
            msg.pose.pose.orientation.z = math.sin(yaw/2)
            msg.pose.pose.orientation.w = math.cos(yaw/2)
            self._pub_odom.publish(msg)
        elif mid == MsgUp.BATTERY:
            d = self._parser.parse_battery(payload)
            if d:
                f = Float32(); f.data = float(d["percent"])
                self._pub_bat.publish(f)
                s = String()
                s.data = f"bat={d['voltage']:.1f}V {d['percent']}% cur={d['current']:.1f}A"
                self._pub_stat.publish(s)
                if d["percent"] < 20:
                    self.get_logger().warn(f"Carrier LOW BATTERY: {d['percent']}%")

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(CarrierBridge())
    rclpy.shutdown()
