#!/usr/bin/env python3
"""Small ROS2 stubs for unit tests that exercise node logic without ROS."""
import sys
import types


class FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


class FakeLogger:
    def __init__(self):
        self.messages = []

    def info(self, msg):
        self.messages.append(("info", msg))

    def warn(self, msg):
        self.messages.append(("warn", msg))

    def error(self, msg):
        self.messages.append(("error", msg))


class _Param:
    def __init__(self, value):
        self.value = value


class _Node:
    def __init__(self, *args, **kwargs):
        self._params = {}
        self._logger = FakeLogger()

    def declare_parameter(self, name, value):
        self._params[name] = value

    def get_parameter(self, name):
        return _Param(self._params.get(name))

    def create_subscription(self, *args, **kwargs):
        return None

    def create_publisher(self, *args, **kwargs):
        return FakePublisher()

    def create_timer(self, *args, **kwargs):
        return None

    def get_logger(self):
        return self._logger

    def get_clock(self):
        return types.SimpleNamespace(
            now=lambda: types.SimpleNamespace(to_msg=lambda: None)
        )


class _ActionClient:
    def __init__(self, *args, **kwargs):
        pass


class _QoSProfile:
    def __init__(self, *args, **kwargs):
        self.args = args
        self.kwargs = kwargs


class _Policy:
    RELIABLE = "reliable"
    BEST_EFFORT = "best_effort"
    KEEP_LAST = "keep_last"
    VOLATILE = "volatile"


class Bool:
    def __init__(self, data=False):
        self.data = data


class Float32:
    def __init__(self, data=0.0):
        self.data = data


class Float32MultiArray:
    def __init__(self, data=None):
        self.data = [] if data is None else data


class String:
    def __init__(self, data=""):
        self.data = data


class Twist:
    def __init__(self):
        self.linear = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)
        self.angular = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)


class PoseStamped:
    def __init__(self):
        self.header = types.SimpleNamespace(stamp=None, frame_id="")
        self.pose = types.SimpleNamespace(
            position=types.SimpleNamespace(x=0.0, y=0.0, z=0.0),
            orientation=types.SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
        )


class OccupancyGrid:
    def __init__(self):
        self.header = types.SimpleNamespace(stamp=None, frame_id="")
        self.info = types.SimpleNamespace(
            resolution=0.0,
            width=0,
            height=0,
            origin=types.SimpleNamespace(
                position=types.SimpleNamespace(x=0.0, y=0.0, z=0.0),
                orientation=types.SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
            ),
        )
        self.data = []


class Odometry:
    def __init__(self):
        self.pose = types.SimpleNamespace(
            pose=types.SimpleNamespace(
                position=types.SimpleNamespace(x=0.0, y=0.0, z=0.0),
                orientation=types.SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
            )
        )
        self.twist = types.SimpleNamespace(
            twist=types.SimpleNamespace(
                linear=types.SimpleNamespace(x=0.0, y=0.0, z=0.0),
                angular=types.SimpleNamespace(x=0.0, y=0.0, z=0.0),
            )
        )


def install_ros_stubs():
    rclpy = types.ModuleType("rclpy")
    rclpy.init = lambda *args, **kwargs: None
    rclpy.spin = lambda *args, **kwargs: None
    rclpy.shutdown = lambda *args, **kwargs: None
    rclpy.ok = lambda: False
    sys.modules.setdefault("rclpy", rclpy)

    rclpy_node = types.ModuleType("rclpy.node")
    rclpy_node.Node = _Node
    sys.modules.setdefault("rclpy.node", rclpy_node)

    rclpy_action = types.ModuleType("rclpy.action")
    rclpy_action.ActionClient = _ActionClient
    sys.modules.setdefault("rclpy.action", rclpy_action)

    rclpy_qos = types.ModuleType("rclpy.qos")
    rclpy_qos.QoSProfile = _QoSProfile
    rclpy_qos.ReliabilityPolicy = _Policy
    rclpy_qos.HistoryPolicy = _Policy
    rclpy_qos.DurabilityPolicy = _Policy
    sys.modules.setdefault("rclpy.qos", rclpy_qos)

    std_msgs = types.ModuleType("std_msgs")
    std_msgs_msg = types.ModuleType("std_msgs.msg")
    std_msgs_msg.Bool = Bool
    std_msgs_msg.Float32 = Float32
    std_msgs_msg.Float32MultiArray = Float32MultiArray
    std_msgs_msg.String = String
    sys.modules.setdefault("std_msgs", std_msgs)
    sys.modules.setdefault("std_msgs.msg", std_msgs_msg)

    geometry_msgs = types.ModuleType("geometry_msgs")
    geometry_msgs_msg = types.ModuleType("geometry_msgs.msg")
    geometry_msgs_msg.PoseStamped = PoseStamped
    geometry_msgs_msg.Twist = Twist
    sys.modules.setdefault("geometry_msgs", geometry_msgs)
    sys.modules.setdefault("geometry_msgs.msg", geometry_msgs_msg)

    nav_msgs = types.ModuleType("nav_msgs")
    nav_msgs_msg = types.ModuleType("nav_msgs.msg")
    nav_msgs_msg.OccupancyGrid = OccupancyGrid
    nav_msgs_msg.Odometry = Odometry
    sys.modules.setdefault("nav_msgs", nav_msgs)
    sys.modules.setdefault("nav_msgs.msg", nav_msgs_msg)
