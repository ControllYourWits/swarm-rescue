#!/usr/bin/env python3
"""DisasterEnv 废墟环境完整测试."""
import os
import sys

import numpy as np

sys.path.insert(
    0, os.path.abspath(os.path.join(os.path.dirname(__file__),
                                     "..", "rl_training")))
from disaster_env import DisasterEnv


# ── 基础接口契约 ───────────────────────────────────────────
def test_observation_shape_and_bounds():
    env = DisasterEnv(enable_frame_stack=False)
    obs, info = env.reset(seed=42)
    assert obs.shape == (43,)
    assert np.all(obs >= -1.0)
    assert np.all(obs <= 1.0)


def test_step_returns_gymnasium_contract():
    env = DisasterEnv(enable_frame_stack=False)
    env.reset(seed=0)
    obs, reward, terminated, truncated, info = env.step(
        np.array([0.5, 0.2], dtype=np.float32))
    assert obs.shape == (43,)
    assert isinstance(reward, float)
    assert np.isfinite(reward)
    assert isinstance(terminated, bool)
    assert isinstance(truncated, bool)
    for key in ("min_dist", "life_conf", "goal_dist"):
        assert key in info


def test_reproducibility():
    env = DisasterEnv(enable_frame_stack=False)
    o1, _ = env.reset(seed=42)
    env = DisasterEnv(enable_frame_stack=False)
    o2, _ = env.reset(seed=42)
    assert np.allclose(o1, o2)


def test_different_seeds_diverge():
    env = DisasterEnv(enable_frame_stack=False)
    o1, _ = env.reset(seed=1)
    env = DisasterEnv(enable_frame_stack=False)
    o2, _ = env.reset(seed=2)
    assert not np.allclose(o1, o2)


# ── 碰撞检测 ───────────────────────────────────────────────
def test_collision_near_debris():
    env = DisasterEnv(enable_frame_stack=False)
    env.reset(seed=1)
    if len(env._obs_r) == 0:
        return
    env._pos[:] = env._obs_c[0] + env._obs_v[0, 0] * 1.01
    lidar = env._sim_lidar_clean()
    assert np.min(lidar) < env.CRASH_DIST * 3


def test_boundary_exit_terminates():
    env = DisasterEnv(enable_frame_stack=False)
    env.reset(seed=0)
    # Place well past the soft boundary margin (0.5m) so termination triggers
    env._pos[0] = env.MAP_SIZE / 2.0 + env.SOFT_BOUNDARY_MARGIN + 0.5
    obs, rew, term, trunc, info = env.step(np.array([0.0, 0.0], dtype=np.float32))
    assert term
    assert info["cause"] == "boundary"


def test_max_steps_truncates():
    env = DisasterEnv(enable_frame_stack=False)
    env.reset(seed=0)
    env._step_n = env.MAX_STEPS - 1
    _, _, _, trunc, info = env.step(np.array([0.0, 0.0], dtype=np.float32))
    assert trunc
    assert info["cause"] == "timeout"


# ── 激光雷达 ───────────────────────────────────────────────
def test_lidar_noise_adds_variance():
    env = DisasterEnv(enable_frame_stack=False)
    env.reset(seed=0)
    clean = env._sim_lidar_clean()
    noisy = env._apply_lidar_noise(clean)
    assert noisy.shape == clean.shape
    assert not np.allclose(noisy, clean)


def test_lidar_bounds():
    env = DisasterEnv(enable_frame_stack=False)
    env.reset(seed=0)
    lidar = env._sim_lidar_clean()
    noisy = env._apply_lidar_noise(lidar)
    assert np.all(noisy >= 0.0)
    assert np.all(noisy <= env.MAX_RANGE)


# ── 多边形障碍物 ───────────────────────────────────────────
def test_polygon_debris_exist():
    env = DisasterEnv(enable_frame_stack=False)
    env.reset(seed=0)
    assert len(env._obs_r) > 0
    assert env._obs_nv[0] >= 4


def test_point_in_polygon():
    env = DisasterEnv(enable_frame_stack=False)
    env._obs_c[0] = np.array([0.0, 0.0], np.float32)
    env._obs_v[0, 0] = [1.0, 0.0]
    env._obs_v[0, 1] = [0.0, 1.0]
    env._obs_v[0, 2] = [-1.0, 0.0]
    env._obs_v[0, 3] = [0.0, -1.0]
    env._obs_nv[0] = 4
    assert env._point_in_polygon(np.array([0.0, 0.0]), env._obs_c[0],
                                 env._obs_v[0], 4)
    assert not env._point_in_polygon(np.array([2.0, 2.0]), env._obs_c[0],
                                     env._obs_v[0], 4)


def test_point_clearance_inside_negative():
    env = DisasterEnv(enable_frame_stack=False)
    env._obs_c[0] = np.array([0.0, 0.0], np.float32)
    env._obs_v[0, 0] = [1.0, 0.0]
    env._obs_v[0, 1] = [0.0, 1.0]
    env._obs_v[0, 2] = [-1.0, 0.0]
    env._obs_v[0, 3] = [0.0, -1.0]
    env._obs_nv[0] = 4
    env._obs_r[0] = 2.0
    assert env._point_clearance(np.array([0.0, 0.0])) < 0.0


# ── 生命体征传感器 ─────────────────────────────────────────
def test_life_sensor_fov():
    env = DisasterEnv(enable_frame_stack=False)
    env.reset(seed=5)
    env._pos[:] = [0.0, 0.0]
    env._yaw = 0.0
    env._obs_c[:] = 99.0
    env._obs_r[:] = 0.1
    env._life[0] = [1.5, 0.0]
    env._life_found = [False] * len(env._life)
    det = [d for d in env._sim_life_sensor() if d["life_id"] == 0]
    assert det
    assert det[0]["confidence"] > 0.2

    env._life[0] = [-1.5, 0.0]
    det = [d for d in env._sim_life_sensor() if d["life_id"] == 0]
    assert not det


def test_life_sensor_occlusion():
    env = DisasterEnv(enable_frame_stack=False)
    env.reset(seed=7)
    env._pos[:] = [0.0, 0.0]
    env._yaw = 0.0
    env._life[0] = [2.0, 0.0]
    env._life_found = [False] * len(env._life)
    env._obs_c[:] = 99.0
    env._obs_r[:] = 0.1
    clear = env._line_occlusion(env._pos, env._life[0])

    env._obs_c[0] = [1.0, 0.0]
    env._obs_r[0] = 0.4
    env._obs_v[0, 0] = [0.4, 0.0]
    env._obs_v[0, 1] = [0.0, 0.4]
    env._obs_v[0, 2] = [-0.4, 0.0]
    env._obs_v[0, 3] = [0.0, -0.4]
    env._obs_nv[0] = 4
    blocked = env._line_occlusion(env._pos, env._life[0])
    assert blocked > clear


def test_found_life_not_detected_again():
    env = DisasterEnv(enable_frame_stack=False)
    env.reset(seed=20)
    env._pos[:] = [0.0, 0.0]
    env._yaw = 0.0
    env._obs_c[:] = 99.0
    env._obs_r[:] = 0.1
    env._life[0] = [1.0, 0.0]
    env._life_found[0] = True
    det = [d for d in env._sim_life_sensor() if d["life_id"] == 0]
    assert not det


def test_multipath_false_positives():
    env = DisasterEnv(enable_frame_stack=False)
    env.reset(seed=0)
    found_false = False
    for _ in range(1000):
        dets = env._sim_life_sensor()
        if any(d["life_id"] is None for d in dets):
            found_false = True
            break
    assert found_false, "multipath false positives should appear occasionally"


# ── 稳定性测试 ─────────────────────────────────────────────
def test_long_episode_stability():
    env = DisasterEnv(enable_frame_stack=False)
    env.reset(seed=0)
    for _ in range(200):
        obs, reward, terminated, truncated, _ = env.step(
            env.action_space.sample())
        assert obs.shape == (43,)
        assert np.isfinite(reward)
        assert np.all(np.isfinite(obs))
        if terminated or truncated:
            env.reset(seed=99)


if __name__ == "__main__":
    test_observation_shape_and_bounds()
    test_step_returns_gymnasium_contract()
    test_reproducibility()
    test_different_seeds_diverge()
    test_collision_near_debris()
    test_boundary_exit_terminates()
    test_max_steps_truncates()
    test_lidar_noise_adds_variance()
    test_lidar_bounds()
    test_polygon_debris_exist()
    test_point_in_polygon()
    test_point_clearance_inside_negative()
    test_life_sensor_fov()
    test_life_sensor_occlusion()
    test_found_life_not_detected_again()
    test_multipath_false_positives()
    test_long_episode_stability()
    print("disaster env tests passed")
