#!/usr/bin/env python3
"""STM32 固件中底盘运动学算法的测试.

验证运动学正逆解, PID 控制器和 Mahony AHRS 姿态估计算法.
逻辑与 Scout STM32 固件的 chassis.c / kinematics.c 保持一致.
"""
import math


# ── 测试辅助函数 (镜像 STM32 kinematics.c) ─────────────────
# 麦克纳姆轮运动学, 4 轮底盘几何参数:
#   a = 半轴距 X (0.160 m), b = 半轴距 Y (0.140 m)
A = 0.160
B = 0.140
WHEEL_RADIUS = 0.048  # M3508 motor wheel radius


def kin_inverse(vx, vy, wz):
    """Mecanum inverse kinematics: chassis velocity -> wheel RPM.

    Returns [front_left, front_right, rear_left, rear_right] in rad/s.
    """
    coeff = 1.0 / WHEEL_RADIUS
    return [
        coeff * (vx - vy - (A + B) * wz),  # FL
        coeff * (vx + vy + (A + B) * wz),  # FR
        coeff * (vx + vy - (A + B) * wz),  # RL
        coeff * (vx - vy + (A + B) * wz),  # RR
    ]


def kin_forward(rpm):
    """Mecanum forward kinematics: wheel RPM -> chassis velocity.

    rpm = [fl, fr, rl, rr] in rad/s.
    Returns (vx, vy, wz) in m/s, m/s, rad/s.
    """
    r = WHEEL_RADIUS / 4.0
    k = 1.0 / (A + B)
    fl, fr, rl, rr = rpm
    vx = r * (fl + fr + rl + rr)
    vy = r * (-fl + fr + rl - rr)
    wz = r * k * (-fl + fr - rl + rr)
    return vx, vy, wz


# ── PID 控制器 (镜像 pid.c) ─────────────────────────────────
class PID:
    def __init__(self, kp, ki, kd, integral_limit, output_limit):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.ki_limit = integral_limit
        self.out_limit = output_limit
        self.integral = 0.0
        self.prev_error = 0.0

    def calc(self, setpoint, measured, dt):
        error = setpoint - measured
        self.integral += error * dt
        self.integral = max(-self.ki_limit, min(self.ki_limit, self.integral))
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        self.prev_error = error
        out = self.kp * error + self.ki * self.integral + self.kd * derivative
        return max(-self.out_limit, min(self.out_limit, out))

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0


# ── Mahony AHRS 姿态估计 (镜像 mahony.c) ───────────────────
def mahony_update(gx, gy, gz, ax, ay, az, q, dt, kp=0.5, ki=0.0):
    """Mahony complementary filter update. Returns (q_new, integral_fb)."""
    q0, q1, q2, q3 = q
    # Normalize accelerometer
    norm = math.sqrt(ax * ax + ay * ay + az * az)
    if norm < 1e-6:
        # integrate gyro only
        dq0 = 0.5 * (-q1 * gx - q2 * gy - q3 * gz)
        dq1 = 0.5 * (q0 * gx + q2 * gz - q3 * gy)
        dq2 = 0.5 * (q0 * gy - q1 * gz + q3 * gx)
        dq3 = 0.5 * (q0 * gz + q1 * gy - q2 * gx)
        return (
            q0 + dq0 * dt, q1 + dq1 * dt, q2 + dq2 * dt, q3 + dq3 * dt
        ), 0.0

    ax /= norm
    ay /= norm
    az /= norm

    # Gravity in body frame from quaternion
    vx = 2.0 * (q1 * q3 - q0 * q2)
    vy = 2.0 * (q0 * q1 + q2 * q3)
    vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3

    # Error = cross(accel, v)
    ex = ay * vz - az * vy
    ey = az * vx - ax * vz
    ez = ax * vy - ay * vx

    # PI correction
    gx_c = gx + kp * ex
    gy_c = gy + kp * ey
    gz_c = gz + kp * ez

    # Integrate quaternion
    dq0 = 0.5 * (-q1 * gx_c - q2 * gy_c - q3 * gz_c)
    dq1 = 0.5 * (q0 * gx_c + q2 * gz_c - q3 * gy_c)
    dq2 = 0.5 * (q0 * gy_c - q1 * gz_c + q3 * gx_c)
    dq3 = 0.5 * (q0 * gz_c + q1 * gy_c - q2 * gx_c)

    q0n = q0 + dq0 * dt
    q1n = q1 + dq1 * dt
    q2n = q2 + dq2 * dt
    q3n = q3 + dq3 * dt

    # Normalize
    norm_q = math.sqrt(q0n * q0n + q1n * q1n + q2n * q2n + q3n * q3n)
    return (q0n / norm_q, q1n / norm_q, q2n / norm_q, q3n / norm_q), 0.0


# ── 测试用例 ───────────────────────────────────────────────
def test_kin_inverse_straight():
    rpm = kin_inverse(0.5, 0.0, 0.0)
    assert all(r > 0 for r in rpm), "all wheels spin forward"
    assert abs(rpm[0] - rpm[1]) < 1e-9
    assert abs(rpm[2] - rpm[3]) < 1e-9


def test_kin_inverse_strafe():
    rpm = kin_inverse(0.0, 0.3, 0.0)
    assert rpm[0] < 0
    assert rpm[1] > 0
    assert rpm[2] > 0
    assert rpm[3] < 0


def test_kin_inverse_rotate():
    rpm = kin_inverse(0.0, 0.0, 1.0)
    assert rpm[0] < 0
    assert rpm[1] > 0
    assert rpm[2] < 0
    assert rpm[3] > 0


def test_kin_roundtrip():
    for vx, vy, wz in [(0.5, 0.0, 0.0), (0.0, 0.3, 0.0),
                        (0.0, 0.0, 1.0), (0.5, 0.3, 0.5),
                        (-0.5, 0.0, 0.0), (0.0, -0.3, 0.0)]:
        rpm = kin_inverse(vx, vy, wz)
        vx_out, vy_out, wz_out = kin_forward(rpm)
        assert abs(vx - vx_out) < 1e-9, f"vx: {vx} vs {vx_out}"
        assert abs(vy - vy_out) < 1e-9, f"vy: {vy} vs {vy_out}"
        assert abs(wz - wz_out) < 1e-9, f"wz: {wz} vs {wz_out}"


def test_kin_forward_zero():
    vx, vy, wz = kin_forward([0.0, 0.0, 0.0, 0.0])
    assert vx == vy == wz == 0.0


def test_kin_inverse_max_speed():
    rpm = kin_inverse(0.6, 0.0, 2.0)
    for r in rpm:
        assert abs(r) < 700.0, f"RPM {r} unrealistic for M3508"


def test_pid_step_response():
    pid = PID(kp=12.0, ki=0.8, kd=0.0,
              integral_limit=2000.0, output_limit=16384.0)
    outputs = []
    measured = 0.0
    for _ in range(100):
        out = pid.calc(100.0, measured, 0.001)
        measured += out * 0.0001
        outputs.append(out)
    assert all(o > 0 for o in outputs), "output should stay positive"
    assert measured > 5.0, "plant should make progress toward setpoint"
    assert measured < 100.0, "should not overshoot setpoint"


def test_pid_reset():
    pid = PID(kp=5.0, ki=1.0, kd=0.0,
              integral_limit=100.0, output_limit=100.0)
    pid.calc(10.0, 0.0, 0.01)
    pid.reset()
    assert pid.integral == 0.0


def test_mahony_level_flight():
    """With no rotation and gravity down, attitude should stay level."""
    q = (1.0, 0.0, 0.0, 0.0)
    for _ in range(100):
        q, _ = mahony_update(0.0, 0.0, 0.0, 0.0, 0.0, 9.81, q, 0.01)
    q0, q1, q2, q3 = q
    # Roll should be near 0
    roll = math.atan2(2.0 * (q2 * q3 + q0 * q1),
                      q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3)
    assert abs(roll) < 0.01


def test_mahony_tilt_detection():
    """Simulate a pitch rotation — attitude should drift from level."""
    q = (1.0, 0.0, 0.0, 0.0)
    for _ in range(100):
        q, _ = mahony_update(0.0, 1.0, 0.0,          # pitch rate 1 rad/s
                              0.0, math.sin(0.3), 9.0, q, 0.01, kp=1.5)
    q0, q1, q2, q3 = q
    pitch = -math.asin(2.0 * (q1 * q3 - q0 * q2))
    assert pitch > 0.3, f"pitch={pitch:.3f} should increase with gyro input"


if __name__ == "__main__":
    test_kin_inverse_straight()
    test_kin_inverse_strafe()
    test_kin_inverse_rotate()
    test_kin_roundtrip()
    test_kin_forward_zero()
    test_kin_inverse_max_speed()
    test_pid_step_response()
    test_pid_reset()
    test_mahony_level_flight()
    test_mahony_tilt_detection()
    print("chassis kinematics tests passed")
