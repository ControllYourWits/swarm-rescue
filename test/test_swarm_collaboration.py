#!/usr/bin/env python3
"""End-to-end collaboration chain across Scout, Carrier, and Specialist."""
from ros_test_stubs import Float32MultiArray

from test_robot_roles import (
    arm_planner,
    life_map_node,
    lora_bridge_node,
    make_arm_planner,
    make_life_map,
    make_supply_manager,
)


def test_life_found_to_supply_and_specialist_delivery_chain():
    scout = make_life_map()
    scout._life_cb(Float32MultiArray(data=[2.0, 0.0, 0.0, 0.92]))
    assert scout._life_map.max() > 0.25

    uplink = lora_bridge_node.pack_lora_uplink(
        lora_bridge_node.ROBOT_ID["scout"],
        lora_bridge_node.STAT_LIFE_FOUND,
        76,
        2.0,
        0.0,
    )
    scout_status = lora_bridge_node.unpack_lora_uplink(uplink)
    assert scout_status["status"] & lora_bridge_node.STAT_LIFE_FOUND

    carrier = make_supply_manager()
    carrier._auto_supply = True
    carrier._life_cb(Float32MultiArray(data=[2.0, 0.0, 0.0, 0.92]))
    assert carrier._pub_cmd.messages[-1].data == "throw:0"

    specialist = make_arm_planner()
    specialist._execute("deliver", arm_planner.ACTIONS["deliver"])
    assert specialist._pub.messages[-1].data == [90.0, 10.0, 10.0, 0.0]

    cx, cy = scout._world2cell(scout_status["pos_x"], scout_status["pos_y"])
    assert life_map_node.LifeMapNode._in_bounds(scout, cx, cy)


if __name__ == "__main__":
    test_life_found_to_supply_and_specialist_delivery_chain()
    print("swarm collaboration tests passed")
