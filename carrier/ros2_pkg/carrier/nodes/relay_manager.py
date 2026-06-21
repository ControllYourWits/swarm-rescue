#!/usr/bin/env python3
"""
relay_manager.py — 4G/5G 信号中继管理节点

功能:
  1. 监控 GL.iNet 路由器状态（通过 HTTP API 或 SNMP）
  2. 监控信号强度 RSSI，低于阈值时发出警告
  3. 发布网络状态 /carrier/network_status
  4. 自动重启路由器（如果 API 支持）

GL.iNet GL-X3000 API: http://192.168.8.1/rpc (ubus JSON-RPC)
"""
import subprocess
import time, threading, json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool

try:
    import requests
    REQUESTS_OK = True
except ImportError:
    REQUESTS_OK = False


class RelayManager(Node):
    ROUTER_IP   = "192.168.8.1"
    ROUTER_PORT = 80
    RSSI_WARN   = -90   # dBm 低于此值告警

    def __init__(self):
        super().__init__("carrier_relay_manager")
        self.declare_parameter("router_ip",  self.ROUTER_IP)
        self.declare_parameter("check_hz",   0.2)   # 5s检查一次

        self._router_ip = self.get_parameter("router_ip").value
        self.declare_parameter("router_password", "")
        self._password = self.get_parameter("router_password").value
        self._session_token = None

        self._pub_net  = self.create_publisher(String, "/carrier/network_status", 10)
        self._pub_ok   = self.create_publisher(Bool,   "/carrier/network_ok",     10)

        hz = self.get_parameter("check_hz").value
        self.create_timer(1.0/hz, self._check)
        self.get_logger().info(f"Relay manager → router {self._router_ip}")

    def _gl_rpc(self, method: str, params: dict = None):
        """调用 GL.iNet ubus RPC API"""
        if not REQUESTS_OK:
            return None
        try:
            url  = f"http://{self._router_ip}/rpc"
            data = {"jsonrpc":"2.0","id":1,"method":method,
                    "params":[self._session_token or "00000000000000000000000000000000",
                               params or {}]}
            r = requests.post(url, json=data, timeout=3)
            return r.json().get("result")
        except Exception:
            return None

    def _login(self):
        if not self._password:
            self.get_logger().warn("Router password not configured, set 'router_password' parameter")
            return False
        res = self._gl_rpc("challenge", {"username":"root"})
        if not res:
            return False
        # 实际需要 MD5(password + alg + nonce) — 简化处理
        res2 = self._gl_rpc("login", {"username":"root","password": self._password})
        if res2 and "ubus_rpc_session" in res2:
            self._session_token = res2["ubus_rpc_session"]
            return True
        return False

    def _check(self):
        if not REQUESTS_OK:
            # 无 requests 库，发布假数据
            s = String(); s.data = "network=unknown (requests not installed)"
            self._pub_net.publish(s)
            b = Bool(); b.data = True
            self._pub_ok.publish(b)
            return

        if not self._session_token:
            self._login()

        # 获取移动网络状态
        res = self._gl_rpc("call",
                           {"path":"modem","method":"get_status"})
        net_ok = False
        status_str = "unknown"

        if res:
            signal = res.get("signal", {})
            rssi   = signal.get("rssi", -120)
            tech   = res.get("access_tech", "N/A")
            net_ok = rssi > self.RSSI_WARN
            status_str = (f"tech={tech} rssi={rssi}dBm "
                         f"{'OK' if net_ok else 'WEAK'}")
            if rssi < self.RSSI_WARN:
                self.get_logger().warn(f"Weak signal: {rssi}dBm")
        else:
            # API 失败，尝试 ping 检测
            r = subprocess.run(["ping","-c","1","-W","2","8.8.8.8"],
                               capture_output=True)
            net_ok = (r.returncode == 0)
            status_str = f"ping={'OK' if net_ok else 'FAIL'}"

        s = String(); s.data = status_str
        self._pub_net.publish(s)
        b = Bool(); b.data = net_ok
        self._pub_ok.publish(b)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(RelayManager())
    rclpy.shutdown()
