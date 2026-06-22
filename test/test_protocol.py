#!/usr/bin/env python3
"""swarm_protocol 通信协议完整测试 (Python 端)."""
import os
import random
import struct
import sys

# 将 swarm_bringup 加入路径, 以便导入 shared 子模块
sys.path.insert(
    0, os.path.abspath(os.path.join(os.path.dirname(__file__),
                                     "..", "swarm_bringup")))
from swarm_bringup.shared.swarm_protocol import (
    FMT_CMD_VEL, FMT_ODOM, FMT_IMU, FMT_STATUS, FMT_BATTERY,
    FMT_ARM_CMD, FMT_SUPPLY_CMD, FMT_LED_CMD,
    FrameParser, MsgDown, MsgUp, RobotId, ChassisMode,
    build_frame, crc8,
    make_arm_cmd, make_cmd_vel, make_heartbeat, make_led_cmd,
    make_set_mode, make_supply_cmd,
)


def feed_frame(parser, frame):
    result = None
    for byte in frame:
        parsed = parser.feed(byte)
        if parsed:
            result = parsed
    return result


# ── CRC8 校验 ───────────────────────────────────────────────
def test_crc8_byte_sized():
    assert 0 <= crc8(b"\x10\x01") <= 255


def test_crc8_zero_data():
    assert crc8(b"") == 0


def test_crc8_known_vectors():
    assert crc8(b"\x00") == 0
    assert crc8(b"\xff") == 0xf3
    assert crc8(b"\x01\x02\x03") == 0x48


def test_crc8_detects_single_bit_flip():
    data = bytes([random.randint(0, 255) for _ in range(10)])
    c1 = crc8(data)
    mutated = bytearray(data)
    mutated[5] ^= 0x01
    assert crc8(bytes(mutated)) != c1


# ── 帧编解码 ────────────────────────────────────────────────
def test_frame_roundtrip():
    payload = struct.pack(FMT_CMD_VEL, 0.3, 0.0, 1.0, 1)
    frame = build_frame(MsgDown.CMD_VEL, payload)
    assert frame[0] == 0xAA
    assert frame[-1] == 0x55
    result = feed_frame(FrameParser(), frame)
    assert result is not None
    msg_id, parsed = result
    assert msg_id == MsgDown.CMD_VEL
    vx, vy, wz, mode = struct.unpack(FMT_CMD_VEL, parsed)
    assert abs(vx - 0.3) < 1e-5
    assert abs(wz - 1.0) < 1e-5
    assert mode == 1


def test_all_command_builders_produce_valid_frames():
    frames = [
        make_cmd_vel(0.5, 0.0, 0.0),
        make_cmd_vel(-0.5, 0.3, -1.0, ChassisMode.NORMAL),
        make_cmd_vel(0.0, 0.0, 0.0, ChassisMode.EMERGENCY),
        make_set_mode(ChassisMode.STOP),
        make_set_mode(ChassisMode.FOLLOW),
        make_heartbeat(),
        make_arm_cmd([45.0, 90.0, 30.0], gripper=1),
        make_arm_cmd([0.0, 0.0, 0.0, 0.0], gripper=0, mode=1),
        make_supply_cmd(2, 1),
        make_supply_cmd(0, 3),
        make_led_cmd(1, 128, 255, 120, 0),
        make_led_cmd(0, 0, 0, 0, 0),
        make_led_cmd(3, 255, 255, 0, 0),
    ]
    for frame in frames:
        assert frame[0] == 0xAA
        assert frame[-1] == 0x55
        assert feed_frame(FrameParser(), frame) is not None


def test_odom_parse():
    payload = struct.pack(FMT_ODOM, 1.5, 0.3, 0.1, 2.0, 3.0, 0.78)
    result = feed_frame(FrameParser(), build_frame(MsgUp.ODOM, payload))
    assert result is not None
    odom = FrameParser().parse_odom(result[1])
    assert abs(odom["vx"] - 1.5) < 1e-5
    assert abs(odom["pos_x"] - 2.0) < 1e-5
    assert abs(odom["yaw"] - 0.78) < 1e-5


def test_imu_parse():
    payload = struct.pack(FMT_IMU,
                          0.1, -9.8, 0.2,  0.01, 0.02, 0.03,
                          0.0, 0.1, 1.57)
    result = feed_frame(FrameParser(), build_frame(MsgUp.IMU, payload))
    assert result is not None
    imu = FrameParser().parse_imu(result[1])
    assert abs(imu["accel"][1] + 9.8) < 1e-5
    assert abs(imu["euler"][2] - 1.57) < 0.01


def test_status_parse():
    payload = struct.pack(FMT_STATUS, RobotId.SCOUT, 1, 0x0F, 0, 24.0)
    result = feed_frame(FrameParser(), build_frame(MsgUp.STATUS, payload))
    assert result is not None
    st = FrameParser().parse_status(result[1])
    assert st["robot_id"] == RobotId.SCOUT
    assert st["motor_ok"] == 0x0F


def test_battery_parse():
    payload = struct.pack(FMT_BATTERY, 24.0, -2.5, 85, 0)
    result = feed_frame(FrameParser(), build_frame(MsgUp.BATTERY, payload))
    assert result is not None
    bat = FrameParser().parse_battery(result[1])
    assert bat["percent"] == 85
    assert bat["charging"] is False


# ── 错误路径 ────────────────────────────────────────────────
def test_bad_crc_frame_dropped():
    frame = bytearray(make_cmd_vel(0.5, 0.0, 0.0))
    frame[-2] ^= 0xFF
    assert feed_frame(FrameParser(), frame) is None


def test_noise_immunity():
    parser = FrameParser()
    random.seed(0)
    noise = bytes([random.randint(0, 255) for _ in range(200)])
    for byte in noise:
        parser.feed(byte)
    result = feed_frame(parser, make_cmd_vel(0.5, 0.0, 0.0))
    assert result is not None


def test_truncated_frame():
    frame = make_cmd_vel(0.5, 0.0, 0.0)
    for cut in range(1, len(frame) - 1):
        assert feed_frame(FrameParser(), frame[:cut]) is None


def test_header_in_payload():
    """Frame with 0xAA inside payload should parse correctly."""
    payload = bytes([0xAA, 0x55, 0x00, 0x01])
    frame = build_frame(MsgDown.HEARTBEAT, payload)
    result = feed_frame(FrameParser(), frame)
    assert result is not None
    assert result[0] == MsgDown.HEARTBEAT


def test_empty_payload():
    frame = build_frame(MsgDown.HEARTBEAT, b"\x00")
    result = feed_frame(FrameParser(), frame)
    assert result is not None


def test_parser_state_reset_after_bad_frame():
    parser = FrameParser()
    bad = bytearray(make_cmd_vel(0.5, 0.0, 0.0))
    bad[-2] ^= 0xFF
    assert feed_frame(parser, bad) is None
    assert feed_frame(parser, make_cmd_vel(0.3, 0.0, 0.5)) is not None


# ── 边界值测试 ──────────────────────────────────────────────
def test_max_speed_values():
    frame = make_cmd_vel(10.0, -10.0, 6.28, ChassisMode.NORMAL)
    result = feed_frame(FrameParser(), frame)
    assert result is not None
    vx, vy, wz, mode = struct.unpack(FMT_CMD_VEL, result[1])
    assert abs(vx - 10.0) < 1e-3


def test_negative_positions():
    payload = struct.pack(FMT_ODOM, 0.0, 0.0, 0.0, -5.0, -5.0, -3.14)
    result = feed_frame(FrameParser(), build_frame(MsgUp.ODOM, payload))
    assert result is not None
    odom = FrameParser().parse_odom(result[1])
    assert odom["pos_x"] < 0
    assert odom["yaw"] < 0


if __name__ == "__main__":
    test_crc8_byte_sized()
    test_crc8_zero_data()
    test_crc8_known_vectors()
    test_crc8_detects_single_bit_flip()
    test_frame_roundtrip()
    test_all_command_builders_produce_valid_frames()
    test_odom_parse()
    test_imu_parse()
    test_status_parse()
    test_battery_parse()
    test_bad_crc_frame_dropped()
    test_noise_immunity()
    test_truncated_frame()
    test_header_in_payload()
    test_empty_payload()
    test_parser_state_reset_after_bad_frame()
    test_max_speed_values()
    test_negative_positions()
    print("protocol tests passed")
