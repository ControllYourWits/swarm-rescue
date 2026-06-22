#!/usr/bin/env python3
"""serial_bridge.py — Specialist 串口桥
发布 /specialist/odom  /specialist/hw_status
订阅 /specialist/cmd_vel  /specialist/arm_cmd  /specialist/led_cmd
"""
import math, threading, time

import rclpy
from rclpy.node import Node
import serial
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Float32MultiArray
from swarm_bringup.shared.swarm_protocol import (
    FrameParser, MsgUp, ChassisMode,
    make_cmd_vel, make_set_mode, make_heartbeat,
    make_arm_cmd, make_led_cmd)
from swarm_bringup.shared.qos_profiles import QOS_SENSOR
from swarm_bringup.shared.robot_state import RobotStateMachine, RobotState

class SpecialistBridge(Node):
    def __init__(self):
        super().__init__("specialist_serial_bridge")
        self.declare_parameter("port", "/dev/ttyS3")
        self.declare_parameter("baud", 921600)
        port = self.get_parameter("port").value
        baud = self.get_parameter("baud").value
        self._ser    = serial.Serial(port, baud, timeout=0.005)
        self._parser = FrameParser()
        self._last_cmd = self.get_clock().now()
        self._last_rx = time.time()
        self._stm32_online = False

        self._pub_odom   = self.create_publisher(Odometry, "/specialist/odom",      QOS_SENSOR)
        self._pub_status = self.create_publisher(String,   "/specialist/hw_status", 10)

        self.create_subscription(Twist,  "/specialist/cmd_vel",
                                 self._cmd_cb,  10)
        self.create_subscription(Float32MultiArray, "/specialist/arm_cmd",
                                 self._arm_cb,  10)
        self.create_subscription(String, "/specialist/led_cmd",
                                 self._led_cb,  10)
        self.create_subscription(String, "/specialist/set_mode",
                                 self._mode_cb, 10)

        self.create_timer(1.0, lambda: self._ser.write(make_heartbeat()))
        self.create_timer(0.1, self._timeout_cb)
        self.create_timer(2.0, self._watchdog_cb)
        self._state_machine = RobotStateMachine("specialist", self)
        threading.Thread(target=self._rx, daemon=True).start()
        self.get_logger().info(f"Specialist bridge {port}@{baud}")

    def _cmd_cb(self, msg):
        self._ser.write(make_cmd_vel(msg.linear.x, msg.linear.y, msg.angular.z))
        self._last_cmd = self.get_clock().now()

    def _arm_cb(self, msg):
        # data: [j0, j1, j2, gripper]
        joints  = list(msg.data[:3]) if len(msg.data) >= 3 else [90.0,10.0,10.0]
        gripper = int(msg.data[3]) if len(msg.data) >= 4 else 0
        self._ser.write(make_arm_cmd(joints, gripper))

    def _led_cb(self, msg):
        # format: "mode:brightness:r:g:b"  e.g. "1:255:255:255:255"
        try:
            parts = [int(x) for x in msg.data.split(":")]
            mode = parts[0] if len(parts)>0 else 0
            brt  = parts[1] if len(parts)>1 else 255
            r    = parts[2] if len(parts)>2 else 255
            g    = parts[3] if len(parts)>3 else 255
            b    = parts[4] if len(parts)>4 else 255
            self._ser.write(make_led_cmd(mode, brt, r, g, b))
        except Exception as e:
            self.get_logger().warn(f"LED cmd parse error: {e}")

    def _mode_cb(self, msg):
        m = {"stop":ChassisMode.STOP,"normal":ChassisMode.NORMAL,
             "emergency":ChassisMode.EMERGENCY}
        self._ser.write(make_set_mode(m.get(msg.data.lower(), ChassisMode.STOP)))

    def _timeout_cb(self):
        if (self.get_clock().now()-self._last_cmd).nanoseconds*1e-9 > 0.8:
            self._ser.write(make_set_mode(ChassisMode.STOP))

    def _watchdog_cb(self):
        elapsed = time.time() - self._last_rx
        was_online = self._stm32_online
        self._stm32_online = elapsed < 3.0
        if was_online and not self._stm32_online:
            self.get_logger().error("Specialist STM32 OFFLINE — no data for 3s")
            self._state_machine.emergency("stm32_offline")
            s = String(); s.data = "OFFLINE"
            self._pub_status.publish(s)
        elif not was_online and self._stm32_online:
            self.get_logger().info("Specialist STM32 recovered")
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
            msg.child_frame_id  = "specialist/base_link"
            msg.pose.pose.position.x = d["pos_x"]
            msg.pose.pose.position.y = d["pos_y"]
            yaw = d["yaw"]
            msg.pose.pose.orientation.z = math.sin(yaw/2)
            msg.pose.pose.orientation.w = math.cos(yaw/2)
            self._pub_odom.publish(msg)
        elif mid == MsgUp.STATUS:
            d = self._parser.parse_status(payload)
            if d:
                s = String()
                s.data = (
                    f"mode={d['mode']} motor={bin(d['motor_ok'])} "
                    f"bat={d['battery_v']:.1f}V"
                )
                self._pub_status.publish(s)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(SpecialistBridge())
    rclpy.shutdown()
