#!/usr/bin/env python3
"""Role-level tests: Scout, Carrier, and Specialist key steps."""
import importlib.util
import os
import sys

import numpy as np

from ros_test_stubs import FakePublisher, Float32MultiArray, String, install_ros_stubs


ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.join(ROOT, "shared"))
install_ros_stubs()


def import_from_path(name, path):
    spec = importlib.util.spec_from_file_location(name, os.path.join(ROOT, path))
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


life_map_node = import_from_path(
    "life_map_node_under_test",
    os.path.join("scout", "ros2_pkg", "scout", "nodes", "life_map_node.py"),
)
lora_bridge_node = import_from_path(
    "lora_bridge_node_under_test",
    os.path.join("carrier", "ros2_pkg", "carrier", "nodes", "lora_bridge_node.py"),
)
supply_manager = import_from_path(
    "supply_manager_under_test",
    os.path.join("carrier", "ros2_pkg", "carrier", "nodes", "supply_manager.py"),
)
thermal_node = import_from_path(
    "thermal_node_under_test",
    os.path.join("specialist", "ros2_pkg", "specialist", "nodes", "thermal_node.py"),
)
arm_planner = import_from_path(
    "arm_planner_under_test",
    os.path.join("specialist", "ros2_pkg", "specialist", "nodes", "arm_planner.py"),
)


def make_life_map():
    node = life_map_node.LifeMapNode.__new__(life_map_node.LifeMapNode)
    node._n = int(node.MAP_SIZE / node.RES)
    node._life_map = np.zeros((node._n, node._n), np.float32)
    node._explored = np.zeros((node._n, node._n), np.bool_)
    node._rx = 0.0
    node._ry = 0.0
    node._ryaw = 0.0
    return node


def make_supply_manager():
    mgr = supply_manager.SupplyManager.__new__(supply_manager.SupplyManager)
    mgr._inventory = {name: True for name in supply_manager.SUPPLY_SLOTS}
    mgr._auto_supply = False
    mgr._pub_cmd = FakePublisher()
    mgr._pub_status = FakePublisher()
    mgr.get_logger = lambda: type("Logger", (), {"warn": lambda self, msg: None})()
    return mgr


def make_arm_planner():
    planner = arm_planner.ArmPlanner.__new__(arm_planner.ArmPlanner)
    planner.STEP_DELAY = 0.0
    planner._busy = False
    planner._current = [90.0, 10.0, 10.0, 0.0]
    planner._pub = FakePublisher()
    planner._pub_status = FakePublisher()
    return planner


def test_scout_life_mapping_step_selects_detected_area():
    scout = make_life_map()
    scout._life_cb(Float32MultiArray(data=[2.0, 0.0, 0.0, 0.9]))
    cx, cy = scout._world2cell(2.0, 0.0)
    assert scout._life_map[cy, cx] > 0.20
    assert scout._patrol_waypoints() in [
        (4.5, 4.0),
        (4.5, -4.0),
        (-4.0, 4.0),
        (-4.0, -4.0),
        (0.0, 5.0),
    ]


def test_carrier_backup_link_and_supply_step():
    frame = lora_bridge_node.pack_lora_uplink(
        lora_bridge_node.ROBOT_ID["carrier"],
        lora_bridge_node.STAT_MOVING,
        87,
        1.25,
        -0.5,
    )
    parsed = lora_bridge_node.unpack_lora_uplink(frame)
    assert parsed["robot_id"] == lora_bridge_node.ROBOT_ID["carrier"]
    assert parsed["battery_pct"] == 87
    assert abs(parsed["pos_x"] - 1.25) < 1e-5

    down = lora_bridge_node.pack_lora_downlink(lora_bridge_node.CMD_RETURN_HOME)
    assert lora_bridge_node.unpack_lora_downlink(down)["cmd_id"] == lora_bridge_node.CMD_RETURN_HOME

    carrier = make_supply_manager()
    carrier._auto_cb(String(data="true"))
    carrier._life_cb(Float32MultiArray(data=[1.8, 0.0, 0.0, 0.85]))
    assert carrier._pub_cmd.messages[-1].data == "throw:0"
    assert carrier._inventory["water"] is False


def test_specialist_thermal_and_arm_step():
    temp = np.full((12, 12), 25.0, np.float32)
    temp[2:5, 3:6] = 36.5
    temp[8:10, 8:11] = 80.0
    persons = thermal_node.ThermalNode._regions(
        thermal_node.ThermalNode.__new__(thermal_node.ThermalNode),
        temp,
        thermal_node.ThermalNode.HUMAN_TEMP_MIN,
        thermal_node.ThermalNode.HUMAN_TEMP_MAX,
    )
    fires = thermal_node.ThermalNode._regions(
        thermal_node.ThermalNode.__new__(thermal_node.ThermalNode),
        temp,
        thermal_node.ThermalNode.FIRE_TEMP_MIN,
        200.0,
    )
    assert len(persons) == 1
    assert len(fires) == 1

    specialist = make_arm_planner()
    specialist._execute("deliver", arm_planner.ACTIONS["deliver"])
    assert len(specialist._pub.messages) == len(arm_planner.ACTIONS["deliver"])
    assert specialist._pub_status.messages[0].data == "RUNNING:deliver"
    assert specialist._pub_status.messages[-1].data == "DONE"


if __name__ == "__main__":
    test_scout_life_mapping_step_selects_detected_area()
    test_carrier_backup_link_and_supply_step()
    test_specialist_thermal_and_arm_step()
    print("robot role tests passed")
