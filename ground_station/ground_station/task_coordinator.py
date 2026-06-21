"""
task_coordinator.py — 竞价式多机任务协调器

参考 Open-RMF 的 bid-based task allocation 模式:
  1. 维护全局任务队列
  2. 当新任务出现 (幸存者检测/物资请求), 计算每个机器人的分配成本
  3. 最低成本机器人获得任务
  4. 支持抢占: 高优先级任务可中断低优先级任务

参考 RoboRTS 的两层架构:
  - 上层: 任务分配与协调 (本节点)
  - 下层: 每个机器人独立执行任务

订阅:
  /scout/life_detections      生命体征检测
  /scout/robot_state          Scout 状态
  /carrier/robot_state        Carrier 状态
  /specialist/robot_state     Specialist 状态
  /carrier/battery            Carrier 电量
  /gs/command                 人工指令覆盖

发布:
  /scout/set_mode             Scout 模式
  /carrier/set_mode           Carrier 模式
  /specialist/set_mode        Specialist 模式
  /specialist/arm_task        Specialist 机械臂任务
  /carrier/supply_request     Carrier 物资投送
  /coordinator/status         协调器状态
"""
import json
import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray, String


class TaskPriority:
    LOW = 0
    NORMAL = 1
    HIGH = 2
    CRITICAL = 3


class Task:
    def __init__(self, task_type, target_x, target_y, priority, data=None):
        self.type = task_type       # "rescue", "deliver", "inspect", "patrol"
        self.target_x = target_x
        self.target_y = target_y
        self.priority = priority
        self.data = data or {}
        self.created_at = time.time()
        self.assigned_to = None
        self.completed = False

    def __repr__(self):
        return (f"Task({self.type}, ({self.target_x:.1f},{self.target_y:.1f}), "
                f"pri={self.priority}, assigned={self.assigned_to})")


class TaskCoordinator(Node):
    """竞价式任务协调器.

    当 Scout 发现幸存者时:
    1. 创建 rescue 任务 (target = 幸存者位置)
    2. 计算 Scout/Carrier/Specialist 到目标的距离成本
    3. 考虑电量、当前状态等因素调整成本
    4. 分配给成本最低且可用的机器人

    成本函数: cost = distance * state_penalty * battery_penalty
      - state_penalty: IDLE=1.0, EXPLORING=1.5, 其他=999(不可用)
      - battery_penalty: >50%=1.0, 20-50%=1.5, <20%=999(不可用)
    """

    RESCUE_CONF_THRESHOLD = 0.65
    DELIVER_CONF_THRESHOLD = 0.75
    TASK_TIMEOUT = 120.0  # 秒

    def __init__(self):
        super().__init__("task_coordinator")

        # Robot states
        self._robots = {
            "scout":      {"x": 0.0, "y": 0.0, "state": "IDLE",
                           "battery": 100.0, "available": True},
            "carrier":    {"x": -1.8, "y": 0.0, "state": "IDLE",
                           "battery": 100.0, "available": True},
            "specialist": {"x": -3.2, "y": -0.2, "state": "IDLE",
                           "battery": 100.0, "available": True},
        }

        self._task_queue = []
        self._active_tasks = []
        self._completed_count = 0

        # Subscriptions - robot states
        for name in self._robots:
            self.create_subscription(
                String, f"/{name}/robot_state",
                self._make_state_cb(name), 10)
            self.create_subscription(
                Float32, f"/{name}/battery",
                self._make_batt_cb(name), 10)

        # Subscriptions - detection events
        self.create_subscription(
            Float32MultiArray, "/scout/life_detections",
            self._life_cb, 10)
        self.create_subscription(
            String, "/swarm/status", self._swarm_cb, 10)
        self.create_subscription(
            String, "/gs/command", self._cmd_cb, 10)

        # Publishers
        self._pub_status = self.create_publisher(
            String, "/coordinator/status", 10)
        self._pub_scout_mode = self.create_publisher(
            String, "/scout/set_mode", 10)
        self._pub_carrier_mode = self.create_publisher(
            String, "/carrier/set_mode", 10)
        self._pub_spec_mode = self.create_publisher(
            String, "/specialist/set_mode", 10)
        self._pub_arm = self.create_publisher(
            String, "/specialist/arm_task", 10)
        self._pub_supply = self.create_publisher(
            String, "/carrier/supply_request", 10)

        self.create_timer(1.0, self._tick)
        self.create_timer(5.0, self._publish_status)
        self.get_logger().info("Task coordinator online (bid-based allocation)")

    def _make_state_cb(self, name):
        def cb(msg):
            parts = msg.data.split("|")
            if parts:
                self._robots[name]["state"] = parts[0]
                self._robots[name]["available"] = parts[0] in ("IDLE", "EXPLORING")
        return cb

    def _make_batt_cb(self, name):
        def cb(msg):
            self._robots[name]["battery"] = float(msg.data)
        return cb

    def _swarm_cb(self, msg):
        try:
            data = json.loads(msg.data)
            for name in ("scout", "carrier", "specialist"):
                if name in data and isinstance(data[name], dict):
                    r = self._robots[name]
                    if "x" in data[name]: r["x"] = float(data[name]["x"])
                    if "y" in data[name]: r["y"] = float(data[name]["y"])
        except (json.JSONDecodeError, KeyError):
            pass

    def _life_cb(self, msg):
        if len(msg.data) < 4:
            return
        dist, breath, heart, conf = [float(x) for x in msg.data[:4]]
        if conf < self.RESCUE_CONF_THRESHOLD:
            return

        # Compute survivor position from scout odom + detection
        scout = self._robots["scout"]
        yaw = 0.0  # Would need scout yaw; use detection range + bearing
        wx = scout["x"] + dist  # Simplified; actual uses yaw
        wy = scout["y"]

        # Determine task type based on confidence
        if conf >= self.DELIVER_CONF_THRESHOLD:
            task_type = "deliver"
            priority = TaskPriority.HIGH
        else:
            task_type = "rescue"
            priority = TaskPriority.NORMAL

        # Avoid duplicate tasks near the same location
        for t in self._active_tasks + self._task_queue:
            if (t.type in ("rescue", "deliver")
                    and math.hypot(t.target_x - wx, t.target_y - wy) < 2.0):
                return

        task = Task(task_type, wx, wy, priority,
                     {"confidence": conf, "range": dist})
        self._task_queue.append(task)
        self.get_logger().info(
            f"New task: {task} (conf={conf:.2f})")

    def _cmd_cb(self, msg):
        """Handle manual commands as high-priority tasks."""
        raw = msg.data.strip()
        parts = raw.split(":")

        if len(parts) >= 3 and parts[1] == "supply":
            # Manual supply request
            task = Task("deliver", 0.0, 0.0, TaskPriority.HIGH,
                         {"item": parts[2], "slot": parts[3] if len(parts) > 3 else "0"})
            self._task_queue.append(task)

    def _tick(self):
        """Main coordination loop: assign queued tasks to robots."""
        # Expire old tasks
        now = time.time()
        self._task_queue = [
            t for t in self._task_queue
            if now - t.created_at < self.TASK_TIMEOUT]
        self._active_tasks = [
            t for t in self._active_tasks
            if now - t.created_at < self.TASK_TIMEOUT]

        # Process task queue
        for task in list(self._task_queue):
            best_robot = self._bid(task)
            if best_robot:
                self._assign(task, best_robot)
                self._task_queue.remove(task)

    def _bid(self, task: Task) -> str:
        """Compute bids from all robots and return the winner.

        Cost function: distance * state_penalty * battery_penalty
        Lower cost = better candidate.
        """
        candidates = {}

        for name, r in self._robots.items():
            if not r["available"]:
                continue
            if r["battery"] < 20.0:
                continue

            # Distance cost
            dist = math.hypot(r["x"] - task.target_x,
                              r["y"] - task.target_y)

            # State penalty
            state_pen = {
                "IDLE": 1.0, "EXPLORING": 1.5,
            }.get(r["state"], 999.0)
            if state_pen >= 999:
                continue

            # Battery penalty
            batt_pen = 1.0 if r["battery"] > 50 else 1.5

            # Role affinity (Scout is better for rescue, Specialist for inspect,
            # Carrier for deliver)
            role_pen = 1.0
            if task.type == "rescue":
                if name == "scout": role_pen = 0.6   # Scout is already there
                if name == "specialist": role_pen = 0.8
            elif task.type == "deliver":
                if name == "carrier": role_pen = 0.3  # Carrier has supplies
                if name == "scout": role_pen = 2.0    # Scout shouldn't deliver
            elif task.type == "inspect":
                if name == "specialist": role_pen = 0.4
                if name == "carrier": role_pen = 2.0

            cost = dist * state_pen * batt_pen * role_pen
            candidates[name] = cost

        if not candidates:
            return None

        winner = min(candidates, key=candidates.get)
        self.get_logger().info(
            f"Bid for {task.type}: {candidates} → {winner}")
        return winner

    def _assign(self, task: Task, robot_name: str):
        """Dispatch task to the winning robot."""
        task.assigned_to = robot_name
        self._active_tasks.append(task)

        r = self._robots[robot_name]
        self.get_logger().info(
            f"Assigned {task.type} to {robot_name} "
            f"(cost={math.hypot(r['x']-task.target_x, r['y']-task.target_y):.1f}m)")

        if task.type == "rescue":
            # Send the assigned robot to investigate
            if robot_name == "scout":
                self._pub_scout_mode.publish(self._make_str("normal"))
            elif robot_name == "specialist":
                self._pub_spec_mode.publish(self._make_str("normal"))
                self._pub_arm.publish(self._make_str("inspect"))

        elif task.type == "deliver":
            if robot_name == "carrier":
                item = task.data.get("item", "water")
                slot = task.data.get("slot", "0")
                self._pub_supply.publish(self._make_str(f"{item}:{slot}"))

        elif task.type == "inspect":
            if robot_name == "specialist":
                self._pub_arm.publish(self._make_str("inspect"))

    def _publish_status(self):
        msg = String()
        msg.data = json.dumps({
            "active_tasks": len(self._active_tasks),
            "queued_tasks": len(self._task_queue),
            "completed": self._completed_count,
            "robots": {name: {
                "state": r["state"],
                "available": r["available"],
                "battery": r["battery"],
            } for name, r in self._robots.items()},
            "tasks": [{
                "type": t.type,
                "assigned": t.assigned_to,
                "target": f"({t.target_x:.1f},{t.target_y:.1f})",
                "priority": t.priority,
            } for t in self._active_tasks],
        })
        self._pub_status.publish(msg)

    @staticmethod
    def _make_str(data: str) -> String:
        s = String()
        s.data = data
        return s


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(TaskCoordinator())
    rclpy.shutdown()
