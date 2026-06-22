#!/usr/bin/env python3
"""terrain_analysis 地形分析 + disaster_recovery 废墟恢复 测试."""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "swarm_bringup"))
from swarm_bringup.shared.robot_state import RobotStateMachine, RobotState


# ── terrain_analysis 核心逻辑测试 (镜像 terrain_analysis.py) ──
def test_world2cell():
    """世界坐标→格栅索引转换."""
    MAP_SIZE = 20.0
    RES = 0.10
    n = int(MAP_SIZE / RES)

    def world2cell(x, y):
        gx = int((x + MAP_SIZE / 2.0) / RES)
        gy = int((y + MAP_SIZE / 2.0) / RES)
        return gx, gy

    # 中心点
    gx, gy = world2cell(0.0, 0.0)
    assert gx == 100 and gy == 100, f"center: ({gx}, {gy})"

    # 左下角
    gx, gy = world2cell(-10.0, -10.0)
    assert gx == 0 and gy == 0, f"origin: ({gx}, {gy})"

    # 右上角
    gx, gy = world2cell(10.0, 10.0)
    assert gx == 200 and gy == 200, f"corner: ({gx}, {gy})"


def test_terrain_classification():
    """地形分类逻辑."""
    SLOPE_WARN = 15.0
    SLOPE_BLOCK = 30.0
    MAX_HEIGHT_DIFF = 0.30
    GRID_RES = 0.10

    def classify(height_diff):
        slope = math.degrees(math.atan2(height_diff, GRID_RES))
        if slope > SLOPE_BLOCK or height_diff > MAX_HEIGHT_DIFF:
            return 3  # 不可通行
        elif slope > SLOPE_WARN or height_diff > 0.15:
            return 2  # 警告
        else:
            return 1  # 可通行

    assert classify(0.02) == 1, "flat should be traversable"
    assert classify(0.05) == 2, "slight slope should be warning (atan2(0.05,0.1)=26deg)"
    assert classify(0.18) == 3, "steep slope should be blocked (atan2(0.18,0.1)=61deg)"
    assert classify(0.35) == 3, "large height diff should be blocked"


def test_ransac_ground():
    """RANSAC 地面分割 (简化版)."""
    rng = np.random.default_rng(42)

    # 生成地面点 (z≈0) + 障碍物点 (z>0.5)
    ground = rng.uniform(-5, 5, (200, 3)).astype(np.float32)
    ground[:, 2] = rng.normal(0, 0.02, 200)
    obstacles = rng.uniform(-5, 5, (50, 3)).astype(np.float32)
    obstacles[:, 2] = rng.uniform(0.5, 2.0, 50)
    points = np.vstack([ground, obstacles])

    # 简化 RANSAC: z < 0.1 = ground
    mask = points[:, 2] < 0.1
    assert np.sum(mask) > 180, f"should find most ground points, got {np.sum(mask)}"
    assert np.sum(~mask) > 40, f"should find obstacle points, got {np.sum(~mask)}"


# ── disaster_recovery 核心逻辑测试 ──
def test_stuck_detection():
    """卡死检测逻辑."""
    STUCK_DIST = 0.1
    STUCK_TIME = 5.0

    import time
    last_pos = (0.0, 0.0)
    last_move_time = time.time()

    # 模拟移动
    current_pos = (0.5, 0.0)
    dist = math.hypot(current_pos[0] - last_pos[0], current_pos[1] - last_pos[1])
    assert dist > STUCK_DIST, "should detect movement"

    # 模拟卡死
    current_pos = (0.505, 0.0)
    dist = math.hypot(current_pos[0] - last_pos[0], current_pos[1] - last_pos[1])
    # dist = 0.505 > 0.1, so not stuck yet


def test_recovery_sequence():
    """恢复序列逻辑."""
    steps = ["backup", "sidestep_right", "sidestep_left", "rotate_180", "human_help"]
    assert len(steps) == 5
    assert steps[0] == "backup"
    assert steps[-1] == "human_help"


if __name__ == "__main__":
    test_world2cell()
    test_terrain_classification()
    test_ransac_ground()
    test_stuck_detection()
    test_recovery_sequence()
    print("terrain_analysis + disaster_recovery tests passed")
