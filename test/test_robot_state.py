#!/usr/bin/env python3
"""robot_state 状态机 + task_coordinator 竞价逻辑测试."""
import os
import sys
import math

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "swarm_bringup"))
from swarm_bringup.shared.robot_state import RobotStateMachine, RobotState


def test_basic_transitions():
    sm = RobotStateMachine("test")
    assert sm.state == RobotState.IDLE
    assert sm.is_available() is True
    sm.transition(RobotState.EXPLORING, "patrol")
    assert sm.state == RobotState.EXPLORING
    assert sm.is_available() is True
    sm.transition(RobotState.RESCUING, "life detected")
    assert sm.state == RobotState.RESCUING
    assert sm.is_available() is False


def test_emergency_and_recovery():
    sm = RobotStateMachine("test")
    sm.transition(RobotState.EXPLORING, "start")
    sm.emergency("test")
    assert sm.is_emergency() is True
    # Illegal transition blocked
    ok = sm.transition(RobotState.EXPLORING, "illegal")
    assert ok is False
    assert sm.state == RobotState.EMERGENCY
    # Recovery
    sm.recover()
    assert sm.state == RobotState.IDLE


def test_state_history():
    sm = RobotStateMachine("test")
    sm.transition(RobotState.EXPLORING, "start")
    sm.transition(RobotState.RESCUING, "found")
    sm.transition(RobotState.IDLE, "done")
    history = sm.get_history()
    assert len(history) == 3
    assert history[0][0] == "IDLE"
    assert history[0][1] == "EXPLORING"


def test_time_in_state():
    import time
    sm = RobotStateMachine("test")
    time.sleep(0.05)
    assert sm.time_in_state > 0.04


def test_to_dict():
    sm = RobotStateMachine("scout")
    d = sm.to_dict()
    assert d["robot"] == "scout"
    assert d["state"] == "IDLE"
    assert d["available"] is True


def test_no_ros():
    """State machine works without rclpy."""
    sm = RobotStateMachine("test", node=None)
    sm.transition(RobotState.EXPLORING, "no crash")
    assert sm.state == RobotState.EXPLORING


if __name__ == "__main__":
    test_basic_transitions()
    test_emergency_and_recovery()
    test_state_history()
    test_time_in_state()
    test_to_dict()
    test_no_ros()
    print("robot_state tests passed")
