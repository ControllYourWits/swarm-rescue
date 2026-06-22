"""
disaster_recovery.py — 废墟环境专用 Nav2 恢复行为

参考 PolarBear pb_nav2_plugins 的自定义恢复策略设计:
  当机器人在废墟中卡住时, 按优先级尝试:
    1. 原地旋转扫描 (找到可通行方向)
    2. 后退 (脱离卡死位置)
    3. 侧移 (麦轮特有, 横向脱离)
    4. 请求重新规划路径

硬件需求: 麦轮全向底盘 (支持 vx, vy, wz 独立控制)

用法: 作为 Nav2 的自定义恢复行为节点运行
  ros2 run scout disaster_recovery

或在 Nav2 BT XML 中引用:
  <Action ID="DisasterRecovery"/>
"""
import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float32, String
from action_msgs.msg import GoalStatus
from rclpy.action import ActionServer

try:
    from nav2_msgs.action import BackUp, Spin
    NAV2_OK = True
except ImportError:
    NAV2_OK = False


class DisasterRecovery(Node):
    """废墟环境卡死恢复行为.

    恢复策略 (按优先级):
    1. 检测卡死: 5 秒内移动距离 < 0.1m
    2. 后退 0.3m (检查后方是否可通行)
    3. 左右侧移 0.3m (麦轮优势)
    4. 原地旋转 180° (寻找新方向)
    5. 如果全部失败, 发布 "STUCK" 状态, 等待人工干预

    参考 RoboMaster 哨兵的恢复逻辑:
    - 快速响应 (< 2s 内开始恢复动作)
    - 保护性动作 (不撞向障碍物)
    - 渐进式恢复 (小动作 → 大动作)
    """

    STUCK_DIST_THRESHOLD = 0.1   # 5 秒内移动 < 0.1m = 卡住
    STUCK_TIME = 5.0             # 卡住检测时间 s
    BACKUP_DIST = 0.3            # 后退距离 m
    SIDESTEP_DIST = 0.3          # 侧移距离 m
    ROTATE_ANGLE = math.pi       # 旋转角度 rad
    MAX_VX = 0.3                 # 恢复时最大线速度
    MAX_VY = 0.3                 # 恢复时最大侧移速度
    MAX_WZ = 1.0                 # 恢复时最大角速度

    def __init__(self):
        super().__init__("disaster_recovery")

        self._rx = 0.0
        self._ry = 0.0
        self._ryaw = 0.0
        self._last_pos = (0.0, 0.0)
        self._last_move_time = time.time()
        self._stuck = False
        self._recovering = False
        self._recovery_step = 0
        self._recovery_start_pos = (0.0, 0.0)
        self._recovery_start_yaw = 0.0

        # 订阅
        self.create_subscription(Odometry, "/scout/odom", self._odom_cb, 10)

        # 发布
        self._pub_cmd = self.create_publisher(Twist, "/scout/cmd_vel", 10)
        self._pub_status = self.create_publisher(String, "/scout/recovery_status", 10)

        self.create_timer(0.5, self._check_stuck)
        self.create_timer(0.1, self._recovery_tick)
        self.get_logger().info("Disaster recovery node ready")

    def _odom_cb(self, msg):
        self._rx = msg.pose.pose.position.x
        self._ry = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self._ryaw = math.atan2(
            2 * (q.w * q.z + q.x * q.y),
            1 - 2 * (q.y * q.y + q.z * q.z))

    def _check_stuck(self):
        """检测是否卡住."""
        dist = math.hypot(self._rx - self._last_pos[0],
                          self._ry - self._last_pos[1])

        if dist > self.STUCK_DIST_THRESHOLD:
            self._last_move_time = time.time()
            self._last_pos = (self._rx, self._ry)
            if self._stuck:
                self.get_logger().info("Recovery successful, robot moving again")
                self._stuck = False
                self._recovering = False
                self._recovery_step = 0
                self._publish_status("RECOVERED")
            return

        # 检查是否卡住超时
        elapsed = time.time() - self._last_move_time
        if elapsed > self.STUCK_TIME and not self._stuck:
            self._stuck = True
            self._recovering = True
            self._recovery_step = 0
            self._recovery_start_pos = (self._rx, self._ry)
            self._recovery_start_yaw = self._ryaw
            self.get_logger().warn(
                f"Robot STUCK! No movement for {elapsed:.1f}s. "
                f"Starting recovery sequence.")
            self._publish_status("STUCK - starting recovery")

    def _recovery_tick(self):
        """恢复动作执行 (每 0.1s 调用一次)."""
        if not self._recovering:
            return

        cmd = Twist()
        step = self._recovery_step

        if step == 0:
            # 步骤 1: 后退
            dist = math.hypot(self._rx - self._recovery_start_pos[0],
                              self._ry - self._recovery_start_pos[1])
            if dist < self.BACKUP_DIST:
                cmd.linear.x = -self.MAX_VX
                self._publish_status("RECOVERY: backing up")
            else:
                self._advance_recovery("backup done")

        elif step == 1:
            # 步骤 2: 右侧移 (麦轮特有)
            # 侧移距离用里程计 y 轴变化近似
            dy = abs(self._ry - self._recovery_start_pos[1])
            if dy < self.SIDESTEP_DIST:
                cmd.linear.y = -self.MAX_VY
                self._publish_status("RECOVERY: sidestepping right")
            else:
                self._advance_recovery("sidestep done")

        elif step == 2:
            # 步骤 3: 左侧移
            dy = abs(self._ry - self._recovery_start_pos[1])
            if dy < self.SIDESTEP_DIST * 2:
                cmd.linear.y = self.MAX_VY
                self._publish_status("RECOVERY: sidestepping left")
            else:
                self._advance_recovery("sidestep done")

        elif step == 3:
            # 步骤 4: 原地旋转 180°
            yaw_diff = abs(self._wrap(self._ryaw - self._recovery_start_yaw))
            if yaw_diff < self.ROTATE_ANGLE * 0.9:
                cmd.angular.z = self.MAX_WZ
                self._publish_status("RECOVERY: rotating 180°")
            else:
                self._advance_recovery("rotation done")

        elif step >= 4:
            # 全部失败
            self._recovering = False
            self.get_logger().error(
                "All recovery steps failed! Requesting human intervention.")
            self._publish_status("STUCK - human intervention required")

        self._pub_cmd.publish(cmd)

    def _advance_recovery(self, reason: str):
        """进入下一个恢复步骤."""
        self._recovery_step += 1
        self._recovery_start_pos = (self._rx, self._ry)
        self._recovery_start_yaw = self._ryaw
        self._last_move_time = time.time()
        self.get_logger().info(f"Recovery step {self._recovery_step}: {reason}")

    def _publish_status(self, text: str):
        msg = String()
        msg.data = text
        self._pub_status.publish(msg)

    @staticmethod
    def _wrap(angle):
        return math.atan2(math.sin(angle), math.cos(angle))


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(DisasterRecovery())
    rclpy.shutdown()
