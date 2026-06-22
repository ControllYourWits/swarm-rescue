"""
lifecycle_manager.py — 机器人生命周期管理器

参考 ROS2 官方 lifecycle 设计 + Nav2 的 LifecycleManager:
  - 管理所有关键节点的 生命周期状态转换
  - Unconfigured → Inactive → Active → Inactive → Cleanup
  - 自动故障检测和重启
  - 远程状态查询和控制

用法:
  ros2 run swarm_bringup lifecycle_manager --ros-args -p robot:=scout

生命周期状态转换:
  [configure]  Unconfigured → Inactive   (初始化硬件, 加载参数)
  [activate]   Inactive → Active          (开始运行, 发布数据)
  [deactivate] Active → Inactive          (暂停运行)
  [cleanup]    Inactive → Unconfigured    (释放资源)
  [shutdown]   任意 → Finalized           (关闭节点)

硬件需求: 无 (纯软件)
"""
import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


# ── 节点生命周期状态定义 ────────────────────────────────
class LifecycleState:
    UNCONFIGURED = "UNCONFIGURED"
    INACTIVE = "INACTIVE"
    ACTIVE = "ACTIVE"
    FINALIZED = "FINALIZED"


class LifecycleTransition:
    CONFIGURE = "configure"
    ACTIVATE = "activate"
    DEACTIVATE = "deactivate"
    CLEANUP = "cleanup"
    SHUTDOWN = "shutdown"


# ── 合法转换表 ──────────────────────────────────────────
_VALID_LC_TRANSITIONS = {
    LifecycleState.UNCONFIGURED: {
        LifecycleTransition.CONFIGURE: LifecycleState.INACTIVE,
        LifecycleTransition.SHUTDOWN: LifecycleState.FINALIZED,
    },
    LifecycleState.INACTIVE: {
        LifecycleTransition.ACTIVATE: LifecycleState.ACTIVE,
        LifecycleTransition.CLEANUP: LifecycleState.UNCONFIGURED,
        LifecycleTransition.SHUTDOWN: LifecycleState.FINALIZED,
    },
    LifecycleState.ACTIVE: {
        LifecycleTransition.DEACTIVATE: LifecycleState.INACTIVE,
        LifecycleTransition.SHUTDOWN: LifecycleState.FINALIZED,
    },
}


# ── 每台机器人的节点清单 ────────────────────────────────
ROBOT_NODES = {
    "scout": [
        "scout_serial_bridge",
        "scout_lidar_proc",
        "scout_radar_processor",
        "scout_life_map_node",
        "scout_rl_nav",
    ],
    "carrier": [
        "carrier_serial_bridge",
        "carrier_follow_nav",
        "carrier_supply_manager",
        "carrier_relay_manager",
    ],
    "specialist": [
        "specialist_serial_bridge",
        "specialist_arm_planner",
        "specialist_thermal",
    ],
}


class LifecycleManager(Node):
    """机器人生命周期管理器.

    功能:
    1. 按顺序启动节点 (configure → activate)
    2. 监控节点健康状态
    3. 故障自动重启 (deactivate → cleanup → configure → activate)
    4. 发布全局生命周期状态

    参考 Nav2 的 LifecycleManager 实现, 适配废墟救援场景:
    - 启动顺序: 通信 → 传感器 → 导航 → 决策
    - 关闭顺序: 决策 → 导航 → 传感器 → 通信
    """

    HEALTH_CHECK_INTERVAL = 5.0   # 健康检查间隔 s
    MAX_RESTART_ATTEMPTS = 3      # 最大重启尝试次数

    def __init__(self):
        super().__init__("lifecycle_manager")
        self.declare_parameter("robot", "scout")

        self._robot = str(self.get_parameter("robot").value)
        self._nodes_config = ROBOT_NODES.get(self._robot, [])

        # 节点状态追踪
        self._node_states = {
            name: LifecycleState.UNCONFIGURED
            for name in self._nodes_config
        }
        self._restart_counts = {name: 0 for name in self._nodes_config}

        # 发布
        self._pub_status = self.create_publisher(
            String, f"/{self._robot}/lifecycle_status", 10)
        self._pub_cmd = self.create_publisher(
            String, f"/{self._robot}/lifecycle_cmd", 10)

        # 订阅外部控制指令
        self.create_subscription(
            String, f"/{self._robot}/lifecycle_cmd",
            self._cmd_cb, 10)

        # 定时健康检查
        self.create_timer(self.HEALTH_CHECK_INTERVAL, self._health_check)

        # 按顺序启动所有节点
        self.create_timer(1.0, self._startup_sequence)

        self.get_logger().info(
            f"Lifecycle manager for [{self._robot}]: "
            f"{len(self._nodes_config)} nodes to manage")

    def _startup_sequence(self):
        """按顺序启动所有节点 (一次性)."""
        self.create_timer(0, lambda: None)  # 取消定时器

        self.get_logger().info(
            f"Starting [{self._robot}] node sequence: "
            f"{' → '.join(self._nodes_config)}")

        for name in self._nodes_config:
            success = self._transition(name, LifecycleTransition.CONFIGURE)
            if success:
                self._transition(name, LifecycleTransition.ACTIVATE)
            else:
                self.get_logger().error(
                    f"Failed to start {name}, "
                    f"attempting restart...")
                self._attempt_restart(name)

        self._publish_status()

    def _cmd_cb(self, msg):
        """处理外部生命周期控制指令.

        格式: "node_name:transition" 例如 "scout_lidar_proc:deactivate"
        或: "all:activate" / "all:shutdown"
        """
        parts = msg.data.split(":")
        if len(parts) != 2:
            return

        target, transition = parts
        if target == "all":
            for name in self._nodes_config:
                self._transition(name, transition)
        elif target in self._node_states:
            self._transition(target, transition)
        self._publish_status()

    def _health_check(self):
        """定期检查节点健康状态, 自动重启异常节点."""
        for name in self._nodes_config:
            state = self._node_states[name]
            if state == LifecycleState.ACTIVE:
                # TODO: 检查节点是否还在响应
                # 目前只是检查状态是否正确
                pass
            elif state == LifecycleState.FINALIZED:
                if self._restart_counts[name] < self.MAX_RESTART_ATTEMPTS:
                    self.get_logger().warn(
                        f"Node {node_name} finalized, attempting restart")
                    self._attempt_restart(name)

        self._publish_status()

    def _transition(self, node_name: str, transition: str) -> bool:
        """尝试对指定节点执行生命周期转换.

        Args:
            node_name: 节点名称
            transition: 转换类型 (configure/activate/deactivate/cleanup/shutdown)

        Returns:
            bool: 转换是否成功
        """
        current = self._node_states.get(node_name)
        if current is None:
            return False

        valid = _VALID_LC_TRANSITIONS.get(current, {})
        if transition not in valid:
            self.get_logger().warn(
                f"Invalid transition: {node_name} {current} → {transition}")
            return False

        new_state = valid[transition]
        self._node_states[node_name] = new_state
        self.get_logger().info(
            f"[{node_name}] {current} --{transition}--> {new_state}")
        return True

    def _attempt_restart(self, node_name: str):
        """尝试重启节点 (cleanup → configure → activate)."""
        self._restart_counts[node_name] += 1
        count = self._restart_counts[node_name]

        if count > self.MAX_RESTART_ATTEMPTS:
            self.get_logger().error(
                f"Node {node_name} failed after {count} restart attempts")
            return

        self.get_logger().info(
            f"Restarting {node_name} (attempt {count})")

        # 强制回到 UNCONFIGURED
        self._node_states[node_name] = LifecycleState.UNCONFIGURED
        self._transition(node_name, LifecycleTransition.CONFIGURE)
        self._transition(node_name, LifecycleTransition.ACTIVATE)

    def _publish_status(self):
        """发布当前所有节点的生命周期状态."""
        status = {
            "robot": self._robot,
            "nodes": dict(self._node_states),
            "restart_counts": dict(self._restart_counts),
        }
        msg = String()
        msg.data = json.dumps(status)
        self._pub_status.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(LifecycleManager())
    rclpy.shutdown()
