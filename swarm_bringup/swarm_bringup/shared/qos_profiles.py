"""QoS profiles for the swarm rescue cluster.

Control commands: Reliable + KeepLast(1) — must not be dropped.
High-frequency sensors: Best Effort — drop stale data rather than retransmit.
"""
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

QOS_COMMAND = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    durability=DurabilityPolicy.VOLATILE,
)

QOS_SENSOR = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    durability=DurabilityPolicy.VOLATILE,
)
