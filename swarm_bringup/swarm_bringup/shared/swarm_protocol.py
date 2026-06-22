"""
swarm_protocol.py
三机协同通信协议 Python 版（与 swarm_protocol.h 完全对应）
"""
import struct
import enum


PROTO_HEADER = 0xAA
PROTO_TAIL   = 0x55

# Robot IDs
class RobotId(enum.IntEnum):
    SCOUT      = 0x01
    CARRIER    = 0x02
    SPECIALIST = 0x03

# Message IDs - uplink (STM32 → RK3588)
class MsgUp(enum.IntEnum):
    ODOM      = 0x01
    IMU       = 0x02
    STATUS    = 0x03
    BATTERY   = 0x04

# Message IDs - downlink (RK3588 → STM32)
class MsgDown(enum.IntEnum):
    CMD_VEL    = 0x10
    SET_MODE   = 0x11
    ARM_CMD    = 0x12
    SUPPLY_CMD = 0x13
    LED_CMD    = 0x14
    HEARTBEAT  = 0x1F

class ChassisMode(enum.IntEnum):
    STOP      = 0x00
    NORMAL    = 0x01
    FOLLOW    = 0x02
    RC        = 0x03
    EMERGENCY = 0xFF

# Struct formats
FMT_CMD_VEL    = '<fffB3x'       # 16B
FMT_ODOM       = '<ffffff'       # 24B
FMT_IMU        = '<fffffffff'    # 36B
FMT_STATUS     = '<BBBBf'        # 8B
FMT_BATTERY    = '<ffBB2x'       # 12B
FMT_ARM_CMD    = '<ffffBB2x'     # 20B
FMT_SUPPLY_CMD = '<BB2x'         # 4B
FMT_LED_CMD    = '<BBBBB3x'      # 8B


def _crc8_table():
    t = []
    for i in range(256):
        crc = i
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if (crc & 0x80) else (crc << 1) & 0xFF
        t.append(crc)
    return t

_CRC8_TABLE = _crc8_table()


def crc8(data: bytes) -> int:
    crc = 0
    for b in data:
        crc = _CRC8_TABLE[crc ^ b]
    return crc


def build_frame(msg_id: int, payload: bytes) -> bytes:
    body = bytes([len(payload) + 1, msg_id]) + payload
    return bytes([PROTO_HEADER]) + body + bytes([crc8(body), PROTO_TAIL])


# ─── 封包函数 ───────────────────────────────────────────────────────
def make_cmd_vel(vx: float, vy: float, wz: float,
                 mode: int = ChassisMode.NORMAL) -> bytes:
    return build_frame(MsgDown.CMD_VEL,
                       struct.pack(FMT_CMD_VEL, vx, vy, wz, mode))

def make_set_mode(mode: int) -> bytes:
    return build_frame(MsgDown.SET_MODE, bytes([mode]))

def make_heartbeat() -> bytes:
    return build_frame(MsgDown.HEARTBEAT, b'\x00')

def make_arm_cmd(joints: list, gripper: int = 0, mode: int = 0) -> bytes:
    j = joints + [0.0] * (4 - len(joints))
    return build_frame(MsgDown.ARM_CMD,
                       struct.pack(FMT_ARM_CMD, j[0], j[1], j[2], j[3], gripper, mode))

def make_supply_cmd(action: int, slot: int = 0) -> bytes:
    return build_frame(MsgDown.SUPPLY_CMD,
                       struct.pack(FMT_SUPPLY_CMD, action, slot))

def make_led_cmd(mode: int, brightness: int = 255,
                 r: int = 255, g: int = 255, b: int = 255) -> bytes:
    return build_frame(MsgDown.LED_CMD,
                       struct.pack(FMT_LED_CMD, mode, brightness, r, g, b))


# ─── 解包类 ────────────────────────────────────────────────────────
class FrameParser:
    def __init__(self):
        self._state  = 'HEADER'
        self._buf    = bytearray()
        self._length = 0

    def feed(self, byte: int):
        if self._state == 'HEADER':
            if byte == PROTO_HEADER:
                self._state = 'LEN'
        elif self._state == 'LEN':
            self._length = byte
            self._buf    = bytearray([byte])
            self._state  = 'DATA'
        elif self._state == 'DATA':
            self._buf.append(byte)
            if len(self._buf) == self._length + 1:
                self._state = 'CRC'
        elif self._state == 'CRC':
            self._state = 'TAIL' if crc8(self._buf) == byte else 'HEADER'
        elif self._state == 'TAIL':
            self._state = 'HEADER'
            if byte == PROTO_TAIL:
                return self._buf[1], bytes(self._buf[2:])
        return None

    def parse_odom(self, payload: bytes) -> dict:
        if len(payload) < struct.calcsize(FMT_ODOM):
            return {}
        vx, vy, wz, px, py, yaw = struct.unpack_from(FMT_ODOM, payload)
        return dict(vx=vx, vy=vy, wz=wz, pos_x=px, pos_y=py, yaw=yaw)

    def parse_imu(self, payload: bytes) -> dict:
        if len(payload) < struct.calcsize(FMT_IMU):
            return {}
        v = struct.unpack_from(FMT_IMU, payload)
        return dict(accel=v[0:3], gyro=v[3:6], euler=v[6:9])

    def parse_status(self, payload: bytes) -> dict:
        if len(payload) < struct.calcsize(FMT_STATUS):
            return {}
        rid, mode, motor, err, batt = struct.unpack_from(FMT_STATUS, payload)
        return dict(robot_id=rid, mode=mode, motor_ok=motor,
                    error_code=err, battery_v=batt)

    def parse_battery(self, payload: bytes) -> dict:
        if len(payload) < struct.calcsize(FMT_BATTERY):
            return {}
        v, c, pct, chg = struct.unpack_from(FMT_BATTERY, payload)
        return dict(voltage=v, current=c, percent=pct, charging=bool(chg))
