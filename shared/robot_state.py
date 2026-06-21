"""
robot_state.py — 三机通用状态机

参考 RoboRTS 的 decision 模块设计:
  - 每个机器人有明确的状态 (IDLE/EXPLORING/RESCUING/DELIVERING/RETURNING/EMERGENCY)
  - 状态转换由事件驱动 (生命体征检测/任务分配/电量低/紧急停止)
  - 发布状态到 /<robot>/robot_state 供协调器和其他机器人参考

用法:
  from robot_state import RobotStateMachine, RobotState

  sm = RobotStateMachine("scout")
  sm.transition(RobotState.EXPLORING)
  print(sm.state, sm.is_available())
"""
import enum
import time

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    RCLPY_OK = True
except ImportError:
    RCLPY_OK = False
    Node = None


class RobotState(enum.Enum):
    """机器人状态定义.

    参考 RoboRTS 决策层的状态划分, 针对废墟救援场景定制:
      IDLE       → 待命, 可接受任务
      EXPLORING  → 执行搜索/导航任务
      RESCUUING  → 正在接近幸存者
      DELIVERING → 正在投送物资
      RETURNING  → 正在返回基地
      EMERGENCY  → 紧急停止/故障
    """
    IDLE = "IDLE"
    EXPLORING = "EXPLORING"
    RESCUING = "RESCUING"
    DELIVERING = "DELIVERING"
    RETURNING = "RETURNING"
    EMERGENCY = "EMERGENCY"


# 合法状态转换表 (参考 RoboRTS 的状态转移规则)
_VALID_TRANSITIONS = {
    RobotState.IDLE: {RobotState.EXPLORING, RobotState.RESCUING,
                      RobotState.DELIVERING, RobotState.RETURNING,
                      RobotState.EMERGENCY},
    RobotState.EXPLORING: {RobotState.IDLE, RobotState.RESCUING,
                           RobotState.RETURNING, RobotState.EMERGENCY},
    RobotState.RESCUING: {RobotState.IDLE, RobotState.DELIVERING,
                          RobotState.EXPLORING, RobotState.EMERGENCY},
    RobotState.DELIVERING: {RobotState.IDLE, RobotState.RETURNING,
                            RobotState.EXPLORING, RobotState.EMERGENCY},
    RobotState.RETURNING: {RobotState.IDLE, RobotState.EXPLORING,
                           RobotState.EMERGENCY},
    RobotState.EMERGENCY: {RobotState.IDLE},
}


class RobotStateMachine:
    """轻量级状态机, 可嵌入任何 ROS2 节点.

    不创建自己的 ROS2 节点; 调用者传入 node 引用来发布状态。

    Args:
        robot_name: 机器人名称 (scout/carrier/specialist)
        node: ROS2 节点引用, 用于发布状态 (可选)
    """

    def __init__(self, robot_name: str, node: Node = None):
        self._name = robot_name
        self._state = RobotState.IDLE
        self._prev_state = RobotState.IDLE
        self._state_time = time.time()
        self._node = node
        self._pub = None
        self._history = []

        if node is not None:
            self._pub = node.create_publisher(
                String, f"/{robot_name}/robot_state", 10)

    @property
    def state(self) -> RobotState:
        return self._state

    @property
    def previous_state(self) -> RobotState:
        return self._prev_state

    @property
    def state_name(self) -> str:
        return self._state.value

    @property
    def time_in_state(self) -> float:
        return time.time() - self._state_time

    def is_available(self) -> bool:
        """是否可接受新任务."""
        return self._state in (RobotState.IDLE, RobotState.EXPLORING)

    def is_emergency(self) -> bool:
        return self._state == RobotState.EMERGENCY

    def transition(self, new_state: RobotState, reason: str = "") -> bool:
        """尝试状态转换.

        Args:
            new_state: 目标状态
            reason: 转换原因 (用于日志)

        Returns:
            bool: 转换是否成功
        """
        if new_state == self._state:
            return True

        allowed = _VALID_TRANSITIONS.get(self._state, set())
        if new_state not in allowed:
            if self._node:
                self._node.get_logger().warn(
                    f"[{self._name}] Illegal transition: "
                    f"{self._state.value} → {new_state.value} "
                    f"(allowed: {[s.value for s in allowed]})")
            return False

        self._prev_state = self._state
        self._state = new_state
        self._state_time = time.time()
        self._history.append(
            (self._prev_state.value, new_state.value, reason,
             self._state_time))

        if len(self._history) > 100:
            self._history = self._history[-50:]

        if self._node:
            self._node.get_logger().info(
                f"[{self._name}] {self._prev_state.value} → "
                f"{new_state.value} ({reason})")

        self._publish()
        return True

    def emergency(self, reason: str = "manual") -> bool:
        """强制进入紧急状态."""
        return self.transition(RobotState.EMERGENCY, reason)

    def recover(self) -> bool:
        """从紧急状态恢复到 IDLE."""
        return self.transition(RobotState.IDLE, "recovery")

    def to_dict(self) -> dict:
        """导出为字典 (用于 JSON 序列化)."""
        return {
            "robot": self._name,
            "state": self._state.value,
            "time_in_state": round(self.time_in_state, 1),
            "available": self.is_available(),
        }

    def get_history(self) -> list:
        """获取最近的状态转换历史."""
        return [h for h in self._history]

    def _publish(self):
        if self._pub:
            msg = String()
            msg.data = (
                f"{self._state.value}|"
                f"{self._prev_state.value}|"
                f"{self.time_in_state:.0f}")
            self._pub.publish(msg)
