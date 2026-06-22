#!/usr/bin/env python3
"""serial_bridge.py — Scout 串口桥节点
订阅 /scout/cmd_vel → STM32
发布 /scout/odom  /scout/imu/raw  /scout/hw_status
"""
import math, threading, time

import rclpy
from rclpy.node import Node
import serial
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import String

from swarm_bringup.shared.swarm_protocol import (
    FrameParser, MsgUp, ChassisMode,
    make_cmd_vel, make_set_mode, make_heartbeat)
from swarm_bringup.shared.qos_profiles import QOS_SENSOR
from swarm_bringup.shared.robot_state import RobotStateMachine, RobotState

class ScoutBridge(Node):
    def __init__(self):
        super().__init__("scout_serial_bridge")
        self.declare_parameter("port", "/dev/ttyS3")
        self.declare_parameter("baud", 921600)
        self.declare_parameter("cmd_timeout", 0.5)
        port = self.get_parameter("port").value
        baud = self.get_parameter("baud").value
        self._to = self.get_parameter("cmd_timeout").value
        self._ser = serial.Serial(port, baud, timeout=0.005)
        self._parser = FrameParser()
        self._last_cmd = self.get_clock().now()
        self._last_rx = time.time()
        self._stm32_online = False

        self._pub_odom   = self.create_publisher(Odometry, "/scout/odom",       QOS_SENSOR)
        self._pub_imu    = self.create_publisher(Imu,      "/scout/imu/raw",    QOS_SENSOR)
        self._pub_status = self.create_publisher(String,   "/scout/hw_status",  10)
        self.create_subscription(Twist,  "/scout/cmd_vel",  self._cmd_cb,  10)
        self.create_subscription(String, "/scout/set_mode", self._mode_cb, 10)
        self.create_timer(1.0, lambda: self._ser.write(make_heartbeat()))
        self.create_timer(0.1, self._timeout_cb)
        self.create_timer(2.0, self._watchdog_cb)
        self._state_machine = RobotStateMachine("scout", self)

    def _heartbeat_cb(self):
        try:
            self._ser.write(make_heartbeat())
        except Exception:
            pass
        threading.Thread(target=self._rx, daemon=True).start()
        self.get_logger().info(f"Scout bridge {port}@{baud}")

    def _cmd_cb(self, msg):
        try:
            self._ser.write(make_cmd_vel(msg.linear.x, msg.linear.y, msg.angular.z))
        except Exception:
            pass
        self._last_cmd = self.get_clock().now()

    def _mode_cb(self, msg):
        m = {"stop":ChassisMode.STOP,"normal":ChassisMode.NORMAL,
             "emergency":ChassisMode.EMERGENCY}
        try:
            self._ser.write(make_set_mode(m.get(msg.data.lower(), ChassisMode.STOP)))
        except Exception:
            pass

    def _timeout_cb(self):
        if (self.get_clock().now()-self._last_cmd).nanoseconds*1e-9 > self._to:
            try:
                self._ser.write(make_set_mode(ChassisMode.STOP))
            except Exception:
                pass

    def _watchdog_cb(self):
        elapsed = time.time() - self._last_rx
        was_online = self._stm32_online
        self._stm32_online = elapsed < 3.0
        if was_online and not self._stm32_online:
            self.get_logger().error("Scout STM32 OFFLINE — no data for 3s")
            self._state_machine.emergency("stm32_offline")
            s = String(); s.data = "OFFLINE"
            self._pub_status.publish(s)
        elif not was_online and self._stm32_online:
            self.get_logger().info("Scout STM32 recovered")
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
            msg = Odometry()
            msg.header.stamp = now; msg.header.frame_id = "odom"
            msg.child_frame_id = "scout/base_link"
            msg.pose.pose.position.x = d["pos_x"]
            msg.pose.pose.position.y = d["pos_y"]
            yaw = d["yaw"]
            msg.pose.pose.orientation.z = math.sin(yaw/2)
            msg.pose.pose.orientation.w = math.cos(yaw/2)
            msg.twist.twist.linear.x  = d["vx"]
            msg.twist.twist.linear.y  = d["vy"]
            msg.twist.twist.angular.z = d["wz"]
            self._pub_odom.publish(msg)
        elif mid == MsgUp.IMU:
            d = self._parser.parse_imu(payload)
            if not d: return
            msg = Imu(); msg.header.stamp = now
            msg.header.frame_id = "scout/imu_link"
            msg.linear_acceleration.x = d["accel"][0]
            msg.linear_acceleration.y = d["accel"][1]
            msg.linear_acceleration.z = d["accel"][2]
            msg.angular_velocity.x = d["gyro"][0]
            msg.angular_velocity.y = d["gyro"][1]
            msg.angular_velocity.z = d["gyro"][2]
            msg.orientation_covariance[0] = -1.0
            self._pub_imu.publish(msg)
        elif mid == MsgUp.STATUS:
            d = self._parser.parse_status(payload)
            if d:
                s = String()
                s.data = f"mode={d['mode']} motor={bin(d['motor_ok'])} bat={d['battery_v']:.2f}V"
                self._pub_status.publish(s)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(ScoutBridge())
    rclpy.shutdown()
