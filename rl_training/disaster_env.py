"""
Scout 废墟救援环境 (DisasterEnv) -- v2.0

基于 CrowdNav, Isaac Lab, Gymnasium-Robotics 等开源项目的最佳实践,
对原始环境进行了 13 项重大改进, 显著提升训练收敛速度和策略质量.

迭代改进清单:
  1.  去除重复常量定义, 统一为单一来源
  2.  航向观测改用 sin/cos 编码, 消除 ±pi 处的不连续性
  3.  引入 PBRS (Potential-Based Reward Shaping), 加速收敛且不改变最优策略
  4.  距离进度奖励归一化, 消除不同初始距离导致的奖励尺度差异
  5.  指数近距惩罚替代线性惩罚, 更有效教机器人保持安全距离
  6.  时序堆叠观测 (3 帧), 提供速度/加速度隐式信息
  7.  课程学习支持: 障碍物数量/密度/地图大小可动态调整
  8.  域随机化增强: 物理参数/噪声参数每回合随机化, 提升 sim-to-real 迁移能力
  9.  动态障碍物: 部分废墟缓慢移动, 增加环境复杂度
  10. 幸存者运动: 幸存者在小范围内缓慢走动, 更贴近真实场景
  11. 探索奖励: 引导机器人覆盖更多未探索区域
  12. 动作平滑惩罚: 惩罚连续动作间的剧烈变化, 生成更平滑的运动轨迹
  13. 软边界处理: 越界不再直接终止, 而是施加递增惩罚并弹回

观测空间 (43 维基础, 3 帧堆叠 = 129 维):
  [0:36]   带噪声的激光雷达扇区, 归一化到 [0, 1]
  [36:38]  底盘速度 [vx/max_vx, wz/max_wz]
  [38]     目标距离 / 地图尺寸
  [39:41]  目标相对航向 sin/cos 编码 (替代原来的 angle/pi)
  [41]     仿真生命体征置信度
  [42]     仿真生命体征距离, 归一化到雷达最大量程

生命探测使用仿真的 FMCW 雷达模型, 包含距离衰减, 方位角衰减, 废墟遮挡,
多径虚警和随机噪声, 确保策略无法学到完美的距离先知.

障碍物使用不规则凸多边形 (4-8 顶点) 替代圆形.
激光雷达包含高斯噪声, 散斑噪声和随机丢点, 更接近真实传感器特性.

参考项目:
  - CrowdNav (UCSB-CARL): 复合奖励 + 课程学习
  - Isaac Lab (NVIDIA): 域随机化 + 地形课程
  - Gymnasium-Robotics (Farama): HER + 目标条件化
  - rl-baselines3-zoo (DLR-RM): 训练稳定性最佳实践
"""
import math
import gymnasium as gym
import numpy as np
from gymnasium import spaces
from collections import deque


class DisasterEnv(gym.Env):
    """废墟救援强化学习训练环境.

    支持课程学习 (curriculum), 时序堆叠 (frame stacking), PBRS 奖励,
    动态障碍物, 幸存者运动等高级特性.
    """
    metadata = {"render_modes": ["human", "rgb_array"]}

    # =====================================================================
    # 迭代 1: 统一常量定义, 消除重复
    # =====================================================================

    # --- 物理参数 ---
    MAX_VX = 0.6             # 最大线速度 m/s
    MAX_WZ = 2.0             # 最大角速度 rad/s
    MAX_STEPS = 1500         # 单回合最大步数
    DT = 0.10                # 仿真步长 s (10Hz 控制频率)

    # --- 传感器参数 ---
    MAX_RANGE = 6.0          # 激光雷达最大量程 m
    N_SECTORS = 36           # 激光雷达扇区数
    RADAR_MAX_RANGE = 5.0    # 毫米波雷达最大量程 m
    RADAR_FOV = math.radians(130.0)  # 雷达视场角

    # --- 碰撞与目标参数 ---
    CRASH_DIST = 0.28        # 碰撞判定距离 m
    NEAR_DIST = 0.60         # 接近警告距离 m
    GOAL_RADIUS = 0.35       # 抵达目标判定半径 m

    # --- 地图参数 (可通过课程学习调整) ---
    MAP_SIZE = 15.0          # 地图边长 m

    # =====================================================================
    # 迭代 7: 课程学习 -- 初始障碍物/幸存者数量为默认值,
    #          实际值在 reset 时由 curriculum_level 决定
    # =====================================================================
    DEFAULT_N_DEBRIS = 18    # 默认废墟障碍物数量
    DEFAULT_N_LIFE = 2       # 默认幸存者数量

    # =====================================================================
    # 迭代 8: 域随机化 -- 噪声参数的基础值, 每回合会在基础值附近随机偏移
    # =====================================================================
    BASE_LIDAR_NOISE_STD = 0.025       # 高斯噪声标准差 m
    BASE_LIDAR_SPECKLE_PROB = 0.03     # 散斑噪声概率
    BASE_LIDAR_SPECKLE_SCALE = 0.35    # 散斑缩放比例
    BASE_LIDAR_DROPOUT_PROB = 0.01     # 丢点概率

    # =====================================================================
    # 迭代 6: 时序堆叠帧数
    # =====================================================================
    N_STACK = 3              # 堆叠帧数 (提供速度/加速度隐式信息)

    # =====================================================================
    # 迭代 10: 幸存者运动参数
    # =====================================================================
    LIFE_WALK_SPEED = 0.15   # 幸存者步行速度 m/s
    LIFE_WALK_RADIUS = 1.5   # 幸存者活动半径 m

    # =====================================================================
    # 迭代 9: 动态障碍物参数
    # =====================================================================
    DYNAMIC_DEBRIS_RATIO = 0.25  # 动态障碍物占总障碍物的比例
    DEBRIS_MAX_SPEED = 0.10      # 动态障碍物最大速度 m/s

    # =====================================================================
    # 迭代 11: 探索网格参数
    # =====================================================================
    EXPLORE_GRID_RES = 0.5   # 探索网格分辨率 m
    EXPLORE_BONUS = 0.3      # 每个新网格的探索奖励

    # =====================================================================
    # 迭代 5: 指数近距惩罚参数
    # =====================================================================
    PROXIMITY_PENALTY_SCALE = 5.0    # 指数惩罚系数
    PROXIMITY_PENALTY_RANGE = 0.60   # 惩罚起始距离 m

    # =====================================================================
    # 迭代 12: 动作平滑惩罚权重
    # =====================================================================
    ACTION_SMOOTH_WEIGHT = 0.05

    # =====================================================================
    # 迭代 13: 软边界参数
    # =====================================================================
    SOFT_BOUNDARY_MARGIN = 0.5       # 软边界缓冲区宽度 m
    SOFT_BOUNDARY_PENALTY_SCALE = 10.0

    # =====================================================================
    # PBRS 参数 (迭代 3)
    # =====================================================================
    PBRS_GAMMA = 0.99        # 与训练 gamma 一致

    def __init__(self, render_mode=None,
                 curriculum_level=1.0,
                 enable_frame_stack=True,
                 enable_dynamic_obstacles=True,
                 enable_moving_survivors=True):
        """初始化环境.

        Args:
            render_mode: 渲染模式, "human" 或 "rgb_array" 或 None
            curriculum_level: 课程学习难度等级 (0.0 ~ 1.0)
                0.0 = 最简单 (少障碍物, 短距离)
                1.0 = 最难 (满障碍物, 长距离)
            enable_frame_stack: 是否启用时序堆叠
            enable_dynamic_obstacles: 是否启用动态障碍物
            enable_moving_survivors: 是否启用幸存者运动
        """
        super().__init__()
        self.render_mode = render_mode
        self._curriculum_level = np.clip(curriculum_level, 0.0, 1.0)
        self._enable_frame_stack = enable_frame_stack
        self._enable_dynamic = enable_dynamic_obstacles
        self._enable_life_walk = enable_moving_survivors

        # =================================================================
        # 迭代 2: 观测空间扩展为 43 维基础 (sin/cos 航向)
        # 迭代 6: 如果启用帧堆叠, 总维度 = 43 * N_STACK
        # =================================================================
        self._base_obs_dim = 43
        if enable_frame_stack:
            total_obs_dim = self._base_obs_dim * self.N_STACK
        else:
            total_obs_dim = self._base_obs_dim

        self.observation_space = spaces.Box(
            -1.0, 1.0, (total_obs_dim,), np.float32)
        self.action_space = spaces.Box(
            -1.0, 1.0, (2,), np.float32)

        self._rng = np.random.default_rng()
        self._screen = None

        # 帧堆叠缓冲区
        self._obs_buffer = None
        if enable_frame_stack:
            self._obs_buffer = deque(maxlen=self.N_STACK)

        self._reset_state()

    def _reset_state(self):
        """重置所有内部状态变量."""
        self._pos = np.zeros(2, np.float32)
        self._yaw = 0.0
        self._vx = 0.0
        self._wz = 0.0
        self._goal = np.zeros(2, np.float32)
        self._obs_c = np.zeros((1, 2), np.float32)    # 障碍物中心 (动态大小)
        self._obs_r = np.zeros(1, np.float32)          # 障碍物半径
        self._obs_v = np.zeros((1, 8, 2), np.float32)  # 障碍物顶点
        self._obs_nv = np.zeros(1, np.int32)            # 障碍物顶点数
        self._life = np.zeros((1, 2), np.float32)       # 幸存者位置
        self._life_found = []
        self._last_life_conf = 0.0
        self._last_life_range = self.RADAR_MAX_RANGE
        self._step_n = 0
        self._prev_dist = 0.0
        self._init_dist = 1.0  # 避免除零

        # 迭代 9: 动态障碍物速度
        self._obs_vel = np.zeros((1, 2), np.float32)
        self._obs_orig_c = np.zeros((1, 2), np.float32)
        self._n_dynamic = 0

        # 迭代 10: 幸存者运动状态
        self._life_orig = np.zeros((1, 2), np.float32)
        self._life_vel = np.zeros((1, 2), np.float32)
        self._life_walk_timer = np.zeros(1, np.float32)

        # 迭代 8: 当前回合的随机化参数
        self._lidar_noise_std = self.BASE_LIDAR_NOISE_STD
        self._lidar_speckle_prob = self.BASE_LIDAR_SPECKLE_PROB
        self._lidar_speckle_scale = self.BASE_LIDAR_SPECKLE_SCALE
        self._lidar_dropout_prob = self.BASE_LIDAR_DROPOUT_PROB
        self._tau = 0.20  # 速度响应时间常数

        # 迭代 11: 探索网格
        self._explore_grid = None
        self._explore_grid_size = 0

        # 迭代 12: 上一步动作 (用于平滑惩罚)
        self._prev_action = np.zeros(2, np.float32)

        # 迭代 3: PBRS 上一步势能值
        self._prev_potential = 0.0

    # =================================================================
    # 课程学习辅助方法 (迭代 7)
    # =================================================================
    def set_curriculum_level(self, level):
        """动态调整课程难度等级.

        Args:
            level: 0.0 (最简单) 到 1.0 (最难)
        """
        self._curriculum_level = float(np.clip(level, 0.0, 1.0))

    def _get_curriculum_params(self):
        """根据课程等级计算当前环境参数.

        Returns:
            dict: 包含 n_debris, n_life, min_goal_dist, obstacle_scale
        """
        lvl = self._curriculum_level
        # 障碍物数量: 6 -> DEFAULT_N_DEBRIS
        n_debris = int(6 + lvl * (self.DEFAULT_N_DEBRIS - 6))
        # 幸存者数量: 1 -> DEFAULT_N_LIFE
        n_life = max(1, int(1 + lvl * (self.DEFAULT_N_LIFE - 1)))
        # 最小目标距离: 2.0 -> 5.0
        min_goal_dist = 2.0 + lvl * 3.0
        # 障碍物尺寸缩放: 0.7 -> 1.0
        obstacle_scale = 0.7 + 0.3 * lvl
        return {
            "n_debris": n_debris,
            "n_life": n_life,
            "min_goal_dist": min_goal_dist,
            "obstacle_scale": obstacle_scale,
        }

    def _update_curriculum_level(self, success_rate):
        """根据成功率自动更新课程等级 (可在训练脚本中调用).

        Args:
            success_rate: 最近一段时间的成功率 (0.0 ~ 1.0)
        """
        # 成功率 > 75% 时提升难度, < 40% 时降低难度
        if success_rate > 0.75 and self._curriculum_level < 1.0:
            self._curriculum_level = min(1.0, self._curriculum_level + 0.05)
        elif success_rate < 0.40 and self._curriculum_level > 0.0:
            self._curriculum_level = max(0.0, self._curriculum_level - 0.03)

    def reset(self, seed=None, options=None):
        """重置环境, 生成新的随机场景.

        根据课程等级调整障碍物数量, 目标距离等参数.
        每回合随机化域参数以提升 sim-to-real 迁移能力.

        Returns:
            obs: 堆叠后的观测向量
            info: 包含课程等级等调试信息
        """
        super().reset(seed=seed)
        if seed is not None:
            self._rng = np.random.default_rng(seed)
        self._reset_state()

        # =================================================================
        # 迭代 7: 获取课程参数
        # =================================================================
        cp = self._get_curriculum_params()
        n_debris = cp["n_debris"]
        n_life = cp["n_life"]
        min_goal_dist = cp["min_goal_dist"]
        obs_scale = cp["obstacle_scale"]

        # 分配正确大小的数组
        self._obs_c = np.zeros((n_debris, 2), np.float32)
        self._obs_r = np.zeros(n_debris, np.float32)
        self._obs_v = np.zeros((n_debris, 8, 2), np.float32)
        self._obs_nv = np.zeros(n_debris, np.int32)
        self._life = np.zeros((n_life, 2), np.float32)
        self._life_found = [False] * n_life

        half = self.MAP_SIZE / 2.0 - 1.5

        # =================================================================
        # 迭代 8: 域随机化 -- 每回合随机化传感器和物理参数
        # 高斯噪声标准差在基础值的 [0.7x, 1.5x] 范围内随机
        # =================================================================
        self._lidar_noise_std = self.BASE_LIDAR_NOISE_STD * float(
            self._rng.uniform(0.7, 1.5))
        self._lidar_speckle_prob = self.BASE_LIDAR_SPECKLE_PROB * float(
            self._rng.uniform(0.5, 2.0))
        self._lidar_dropout_prob = self.BASE_LIDAR_DROPOUT_PROB * float(
            self._rng.uniform(0.3, 2.5))
        # 物理时间常数在 [0.15, 0.30] 之间随机 (模拟不同地面摩擦)
        self._tau = float(self._rng.uniform(0.15, 0.30))

        # =================================================================
        # 生成障碍物 (凸多边形)
        # =================================================================
        for j in range(n_debris):
            self._obs_c[j] = self._rng.uniform(-half, half, 2).astype(np.float32)
            radius = float(self._rng.uniform(0.20, 0.70)) * obs_scale
            self._obs_r[j] = radius
            nv = int(self._rng.integers(4, 9))
            self._obs_nv[j] = nv
            angles = np.sort(self._rng.uniform(0, 2 * math.pi, nv))
            for k, a in enumerate(angles):
                r_var = radius * float(self._rng.uniform(0.55, 1.0))
                self._obs_v[j, k, 0] = r_var * math.cos(a)
                self._obs_v[j, k, 1] = r_var * math.sin(a)

        # =================================================================
        # 迭代 9: 标记动态障碍物并初始化速度
        # 选前 n_dynamic 个障碍物为动态的
        # =================================================================
        self._n_dynamic = max(1, int(n_debris * self.DYNAMIC_DEBRIS_RATIO))
        self._obs_vel = np.zeros((n_debris, 2), np.float32)
        self._obs_orig_c = self._obs_c.copy()
        for j in range(self._n_dynamic):
            # 随机运动方向和速度
            angle = float(self._rng.uniform(0, 2 * math.pi))
            speed = float(self._rng.uniform(0.03, self.DEBRIS_MAX_SPEED))
            self._obs_vel[j, 0] = speed * math.cos(angle)
            self._obs_vel[j, 1] = speed * math.sin(angle)

        # =================================================================
        # 生成机器人起始位置 (确保与障碍物有足够间距)
        # =================================================================
        for _ in range(300):
            p = self._rng.uniform(-half, half, 2).astype(np.float32)
            if self._point_clearance(p, n_debris) > 0.8:
                self._pos = p
                break

        # =================================================================
        # 生成目标位置 (确保与起始点有足够距离)
        # =================================================================
        for _ in range(300):
            g = self._rng.uniform(-half, half, 2).astype(np.float32)
            if (float(np.linalg.norm(g - self._pos)) > min_goal_dist
                    and self._point_clearance(g, n_debris) > 0.5):
                self._goal = g
                break

        # =================================================================
        # 生成幸存者位置
        # =================================================================
        self._life_orig = np.zeros((n_life, 2), np.float32)
        self._life_vel = np.zeros((n_life, 2), np.float32)
        self._life_walk_timer = np.zeros(n_life, np.float32)
        for i in range(n_life):
            for _ in range(200):
                lp = self._rng.uniform(-half, half, 2).astype(np.float32)
                if (self._point_clearance(lp, n_debris) > 0.3
                        and float(np.linalg.norm(lp - self._pos)) > 2.0):
                    self._life[i] = lp
                    self._life_orig[i] = lp.copy()
                    break
            # 初始化随机步行方向
            walk_angle = float(self._rng.uniform(0, 2 * math.pi))
            self._life_vel[i, 0] = self.LIFE_WALK_SPEED * math.cos(walk_angle)
            self._life_vel[i, 1] = self.LIFE_WALK_SPEED * math.sin(walk_angle)
            self._life_walk_timer[i] = float(self._rng.uniform(2.0, 8.0))

        self._yaw = float(self._rng.uniform(-math.pi, math.pi))
        self._prev_dist = float(np.linalg.norm(self._goal - self._pos))
        self._init_dist = max(self._prev_dist, 1.0)

        # =================================================================
        # 迭代 11: 初始化探索网格
        # =================================================================
        grid_size = int(self.MAP_SIZE / self.EXPLORE_GRID_RES) + 1
        self._explore_grid_size = grid_size
        self._explore_grid = np.zeros((grid_size, grid_size), dtype=np.bool_)

        # 标记起始位置附近为已探索
        gx, gy = self._pos_to_grid(self._pos)
        r_cells = max(1, int(1.0 / self.EXPLORE_GRID_RES))
        self._mark_explored(gx, gy, r_cells)

        # 迭代 12: 重置上一步动作
        self._prev_action = np.zeros(2, np.float32)

        # 迭代 3: 计算初始 PBRS 势能
        self._prev_potential = -self._prev_dist / self.MAP_SIZE

        # =================================================================
        # 迭代 6: 填充帧堆叠缓冲区 (用零向量填充)
        # =================================================================
        if self._enable_frame_stack and self._obs_buffer is not None:
            self._obs_buffer.clear()
            zero_frame = np.zeros(self._base_obs_dim, np.float32)
            for _ in range(self.N_STACK):
                self._obs_buffer.append(zero_frame)

        obs = self._get_obs()
        info = {"curriculum_level": self._curriculum_level}
        return obs, info

    # =================================================================
    # 多边形碰撞检测辅助函数 (保持不变)
    # =================================================================
    @staticmethod
    def _point_in_polygon(pt, centre, verts, nv):
        """判断点是否在凸多边形内部 (射线法).

        Args:
            pt: 待检测的点 [x, y]
            centre: 多边形中心坐标
            verts: 顶点相对坐标数组
            nv: 顶点数量

        Returns:
            bool: 点是否在多边形内部
        """
        x, y = pt[0] - centre[0], pt[1] - centre[1]
        inside = False
        for k in range(nv):
            x0, y0 = verts[k, 0], verts[k, 1]
            x1, y1 = verts[(k + 1) % nv, 0], verts[(k + 1) % nv, 1]
            if ((y0 > y) != (y1 > y)) and (x < (x1 - x0) * (y - y0) / (y1 - y0) + x0):
                inside = not inside
        return inside

    def _point_clearance(self, pt, n_debris=None):
        """计算点到最近障碍物的距离.

        Args:
            pt: 待检测的点 [x, y]
            n_debris: 障碍物数量 (None 则用当前数组长度)

        Returns:
            float: 到最近障碍物的距离, 负值表示在障碍物内部
        """
        if n_debris is None:
            n_debris = len(self._obs_r)
        best = self.MAP_SIZE
        for j in range(n_debris):
            d_centre = float(np.linalg.norm(pt - self._obs_c[j]))
            if d_centre > self._obs_r[j] + best:
                continue
            nv = self._obs_nv[j]
            if self._point_in_polygon(pt, self._obs_c[j], self._obs_v[j], nv):
                return -0.01
            for k in range(nv):
                a = self._obs_c[j] + self._obs_v[j, k]
                b = self._obs_c[j] + self._obs_v[j, (k + 1) % nv]
                ab = b - a
                ab2 = float(np.dot(ab, ab))
                if ab2 < 1e-9:
                    d = float(np.linalg.norm(pt - a))
                else:
                    t = np.clip(float(np.dot(pt - a, ab)) / ab2, 0.0, 1.0)
                    d = float(np.linalg.norm(pt - (a + t * ab)))
                if d < best:
                    best = d
        return best

    # =================================================================
    # 探索网格辅助方法 (迭代 11)
    # =================================================================
    def _pos_to_grid(self, pos):
        """将世界坐标转换为探索网格索引.

        Args:
            pos: 世界坐标 [x, y]

        Returns:
            tuple: (grid_x, grid_y) 网格索引
        """
        gx = int((pos[0] + self.MAP_SIZE / 2.0) / self.EXPLORE_GRID_RES)
        gy = int((pos[1] + self.MAP_SIZE / 2.0) / self.EXPLORE_GRID_RES)
        gx = np.clip(gx, 0, self._explore_grid_size - 1)
        gy = np.clip(gy, 0, self._explore_grid_size - 1)
        return int(gx), int(gy)

    def _mark_explored(self, cx, cy, radius_cells):
        """标记指定网格位置周围的区域为已探索.

        Args:
            cx: 中心网格 x 索引
            cy: 中心网格 y 索引
            radius_cells: 标记半径 (网格单元数)
        """
        if self._explore_grid is None:
            return
        gs = self._explore_grid_size
        for dx in range(-radius_cells, radius_cells + 1):
            for dy in range(-radius_cells, radius_cells + 1):
                if dx * dx + dy * dy <= radius_cells * radius_cells:
                    nx, ny = cx + dx, cy + dy
                    if 0 <= nx < gs and 0 <= ny < gs:
                        self._explore_grid[nx, ny] = True

    def _check_explore_bonus(self):
        """检查当前位置是否进入新的探索网格, 并计算探索奖励.

        Returns:
            float: 探索奖励 (新网格 > 0, 已探索过 = 0)
        """
        gx, gy = self._pos_to_grid(self._pos)
        if self._explore_grid is not None and not self._explore_grid[gx, gy]:
            r_cells = max(1, int(0.5 / self.EXPLORE_GRID_RES))
            self._mark_explored(gx, gy, r_cells)
            return self.EXPLORE_BONUS
        return 0.0

    # =================================================================
    # 迭代 9: 动态障碍物更新
    # =================================================================
    def _update_dynamic_obstacles(self):
        """更新动态障碍物的位置.

        动态障碍物在原始位置附近做缓慢的随机游走,
        当超出活动范围时自动反弹.
        """
        for j in range(self._n_dynamic):
            # 更新位置
            self._obs_c[j, 0] += self._obs_vel[j, 0] * self.DT
            self._obs_c[j, 1] += self._obs_vel[j, 1] * self.DT

            # 检查是否超出原始位置的活动范围 (2m)
            diff = self._obs_c[j] - self._obs_orig_c[j]
            dist_from_orig = float(np.linalg.norm(diff))
            if dist_from_orig > 2.0:
                # 反弹: 速度方向朝原始位置
                direction = -diff / (dist_from_orig + 1e-6)
                speed = float(np.linalg.norm(self._obs_vel[j]))
                # 加一点随机扰动
                perturb = self._rng.uniform(-0.3, 0.3)
                angle = math.atan2(direction[1], direction[0]) + perturb
                self._obs_vel[j, 0] = speed * math.cos(angle)
                self._obs_vel[j, 1] = speed * math.sin(angle)

            # 随机速度微调 (每 50 步)
            if self._step_n % 50 == 0 and self._rng.random() < 0.3:
                angle = float(self._rng.uniform(0, 2 * math.pi))
                speed = float(self._rng.uniform(0.02, self.DEBRIS_MAX_SPEED))
                self._obs_vel[j, 0] = speed * math.cos(angle)
                self._obs_vel[j, 1] = speed * math.sin(angle)

    # =================================================================
    # 迭代 10: 幸存者运动更新
    # =================================================================
    def _update_survivor_movement(self):
        """更新幸存者位置 (模拟缓慢步行).

        幸存者在原始位置附近缓慢走动, 每隔一段时间改变方向.
        """
        if not self._enable_life_walk:
            return
        for i in range(len(self._life)):
            if self._life_found[i]:
                continue
            # 倒计时
            self._life_walk_timer[i] -= self.DT
            if self._life_walk_timer[i] <= 0:
                # 切换方向
                angle = float(self._rng.uniform(0, 2 * math.pi))
                self._life_vel[i, 0] = self.LIFE_WALK_SPEED * math.cos(angle)
                self._life_vel[i, 1] = self.LIFE_WALK_SPEED * math.sin(angle)
                self._life_walk_timer[i] = float(self._rng.uniform(3.0, 10.0))

            # 更新位置
            self._life[i, 0] += self._life_vel[i, 0] * self.DT
            self._life[i, 1] += self._life_vel[i, 1] * self.DT

            # 检查是否超出活动范围
            diff = self._life[i] - self._life_orig[i]
            if float(np.linalg.norm(diff)) > self.LIFE_WALK_RADIUS:
                # 朝原始位置方向返回
                direction = -diff / (float(np.linalg.norm(diff)) + 1e-6)
                self._life_vel[i, 0] = self.LIFE_WALK_SPEED * direction[0]
                self._life_vel[i, 1] = self.LIFE_WALK_SPEED * direction[1]

    # =================================================================
    # 单步仿真
    # =================================================================
    def step(self, action):
        """执行单步仿真, 计算奖励和终止条件.

        Args:
            action: 二维动作向量 [vx_cmd, wz_cmd], 范围 [-1, 1]

        Returns:
            obs: 堆叠后的观测向量
            reward: 当前步的奖励值
            terminated: 是否因碰撞/目标达成而终止
            truncated: 是否因超时而截断
            info: 调试信息字典
        """
        action = np.clip(action, -1.0, 1.0)
        vx_cmd = float(action[0]) * self.MAX_VX
        wz_cmd = float(action[1]) * self.MAX_WZ

        # 一阶速度动力学 (带域随机化的时间常数)
        k = self.DT / self._tau
        self._vx += k * (vx_cmd - self._vx)
        self._wz += k * (wz_cmd - self._wz)

        # 航向积分 + 位置积分
        self._yaw += self._wz * self.DT
        self._yaw = math.atan2(math.sin(self._yaw), math.cos(self._yaw))
        self._pos[0] += self._vx * math.cos(self._yaw) * self.DT
        self._pos[1] += self._vx * math.sin(self._yaw) * self.DT
        self._step_n += 1

        # 迭代 9: 更新动态障碍物
        if self._enable_dynamic:
            self._update_dynamic_obstacles()

        # 迭代 10: 更新幸存者位置
        self._update_survivor_movement()

        lidar_raw = self._sim_lidar_clean()
        lidar = self._apply_lidar_noise(lidar_raw)
        min_d = float(np.min(lidar))
        curr_d = float(np.linalg.norm(self._goal - self._pos))

        # =================================================================
        # 迭代 13: 软边界处理
        # 越界时施加递增惩罚而非直接终止, 允许机器人自行修正
        # =================================================================
        half = self.MAP_SIZE / 2.0
        boundary_penalty = 0.0
        terminated = False
        truncated = False
        info = {}

        overshoot_x = abs(self._pos[0]) - half
        overshoot_y = abs(self._pos[1]) - half
        if overshoot_x > 0 or overshoot_y > 0:
            max_overshoot = max(overshoot_x, overshoot_y)
            if max_overshoot > self.SOFT_BOUNDARY_MARGIN:
                # 超出缓冲区太远, 直接终止
                return self._get_obs(), -50.0, True, False, {"cause": "boundary"}
            # 在缓冲区内, 施加二次惩罚并弹回
            boundary_penalty = -self.SOFT_BOUNDARY_PENALTY_SCALE * (max_overshoot ** 2)
            # 弹回力: 将位置拉回边界内
            if overshoot_x > 0:
                sign_x = 1.0 if self._pos[0] > 0 else -1.0
                self._pos[0] = sign_x * (half - 0.01)
            if overshoot_y > 0:
                sign_y = 1.0 if self._pos[1] > 0 else -1.0
                self._pos[1] = sign_y * (half - 0.01)

        reward = 0.0

        # =================================================================
        # 迭代 4: 归一化距离进度奖励
        # 用 (d_prev - d_curr) / init_dist 归一化, 使奖励尺度不依赖初始距离
        # =================================================================
        progress = (self._prev_dist - curr_d) / self._init_dist
        reward += 5.0 * progress
        self._prev_dist = curr_d

        # =================================================================
        # 迭代 3: PBRS (Potential-Based Reward Shaping)
        # F(s,s') = gamma * Phi(s') - Phi(s), 其中 Phi = -dist / map_size
        # 理论保证: PBRS 不改变最优策略, 但显著加速收敛
        # 参考: Ng et al. "Policy Invariance Under Reward Transformations" (1999)
        # =================================================================
        current_potential = -curr_d / self.MAP_SIZE
        pbrs_reward = self.PBRS_GAMMA * current_potential - self._prev_potential
        reward += pbrs_reward
        self._prev_potential = current_potential

        # =================================================================
        # 迭代 2: sin/cos 航向奖励 (替代原来的 |ga|/pi)
        # sin/cos 编码避免了 -pi 到 pi 的跳变, 更适合神经网络学习
        # =================================================================
        gv = self._goal - self._pos
        ga = math.atan2(gv[1], gv[0]) - self._yaw
        ga = math.atan2(math.sin(ga), math.cos(ga))
        # 使用 cos(ga) 作为航向奖励: 正对目标时为 1, 背对目标时为 -1
        reward += 0.10 * math.cos(ga)

        # 生命探测奖励 (保持不变)
        detections = self._sim_life_sensor()
        n_debris = len(self._obs_r)
        self._last_life_conf = max((d["confidence"] for d in detections), default=0.0)
        self._last_life_range = min(
            (d["range"] for d in detections), default=self.RADAR_MAX_RANGE)

        for det in detections:
            life_id = det["life_id"]
            if life_id is None or self._life_found[life_id]:
                continue
            reward += 8.0 * det["confidence"]
            if det["confidence"] >= 0.72:
                reward += 45.0
                self._life_found[life_id] = True
                info["life_found"] = life_id

        # =================================================================
        # 目标达成/碰撞判定
        # =================================================================
        if curr_d < self.GOAL_RADIUS:
            reward += 200.0
            terminated = True
            info["cause"] = "goal_reached"
        elif min_d < self.CRASH_DIST:
            reward -= 100.0
            terminated = True
            info["cause"] = "collision"

        # =================================================================
        # 迭代 5: 指数近距惩罚
        # 使用 exp(-alpha * (d - crash_dist)) 产生急剧增长的惩罚梯度,
        # 比原来的 2/(d+0.01) 更有效引导机器人远离障碍物
        # =================================================================
        if not terminated and min_d < self.PROXIMITY_PENALTY_RANGE:
            excess = self.PROXIMITY_PENALTY_RANGE - min_d
            # 惩罚值: 距离越近, 惩罚越大, 且增长越快
            proximity_penalty = self.PROXIMITY_PENALTY_SCALE * math.exp(
                -5.0 * (min_d - self.CRASH_DIST)) * excess
            reward -= proximity_penalty

        # =================================================================
        # 迭代 11: 探索奖励
        # =================================================================
        explore_bonus = self._check_explore_bonus()
        reward += explore_bonus

        # =================================================================
        # 迭代 12: 动作平滑惩罚
        # 惩罚连续动作间的剧烈变化, 鼓励平滑运动
        # 参考: CrowdNav, Gym-Gazebo2 的角速度惩罚
        # =================================================================
        action_diff = float(np.linalg.norm(action - self._prev_action))
        reward -= self.ACTION_SMOOTH_WEIGHT * action_diff
        self._prev_action = action.copy()

        # 基础时间惩罚 (鼓励高效完成任务)
        reward -= 0.01

        # 迭代 13: 软边界惩罚
        reward += boundary_penalty

        # 超时判定
        if self._step_n >= self.MAX_STEPS:
            truncated = True
            info["cause"] = "timeout"

        obs = self._get_obs()
        info.update({
            "min_dist": min_d,
            "goal_dist": curr_d,
            "life_conf": self._last_life_conf,
            "life_range": self._last_life_range,
            "life_found_count": sum(self._life_found),
            "curriculum_level": self._curriculum_level,
            "explore_bonus": explore_bonus,
            "pbrs_reward": pbrs_reward,
            "progress_reward": 5.0 * progress,
            "obs_dim": obs.shape[0],
        })
        return obs, float(reward), terminated, truncated, info

    # =================================================================
    # 激光雷达仿真 (保持核心逻辑不变, 使用当前回合的随机化参数)
    # =================================================================
    def _sim_lidar_clean(self):
        """计算无噪声的激光雷达距离数据.

        对每个扇区发射射线, 计算与所有多边形障碍物边及地图边界的最近交点.

        Returns:
            np.ndarray: N_SECTORS 个距离值, 单位 m
        """
        n_debris = len(self._obs_r)
        dists = np.full(self.N_SECTORS, self.MAX_RANGE, np.float32)
        angles = (np.linspace(0, 2 * math.pi, self.N_SECTORS, endpoint=False)
                  + self._yaw)
        half = self.MAP_SIZE / 2.0

        for b in range(self.N_SECTORS):
            dx, dy = math.cos(angles[b]), math.sin(angles[b])
            # 与障碍物多边形的边求交
            for j in range(n_debris):
                nv = self._obs_nv[j]
                for k in range(nv):
                    a = self._obs_c[j] + self._obs_v[j, k]
                    bp = self._obs_c[j] + self._obs_v[j, (k + 1) % nv]
                    t = self._ray_seg_intersect(self._pos, (dx, dy), a, bp)
                    if 0.01 < t < dists[b]:
                        dists[b] = float(t)
            # 与地图边界求交
            for s, ax in [(1, 0), (-1, 0), (1, 1), (-1, 1)]:
                wall = s * half
                d = dx if ax == 0 else dy
                p = self._pos[ax]
                if abs(d) > 1e-9:
                    t = (wall - p) / d
                    if 0.01 < t < dists[b]:
                        dists[b] = float(t)
        return dists

    @staticmethod
    def _ray_seg_intersect(origin, direction, seg_a, seg_b):
        """计算射线与线段的交点参数 t.

        Args:
            origin: 射线起点 [x, y]
            direction: 射线方向 (dx, dy), 不必归一化
            seg_a: 线段起点 [x, y]
            seg_b: 线段终点 [x, y]

        Returns:
            float: 交点参数 t (沿射线方向的距离), 无交点返回 inf
        """
        ox, oy = origin[0], origin[1]
        dx, dy = direction[0], direction[1]
        ax, ay = seg_a[0], seg_a[1]
        bx, by = seg_b[0], seg_b[1]
        denom = dx * (ay - by) - dy * (ax - bx)
        if abs(denom) < 1e-12:
            return float("inf")
        t_num = (ox - ax) * (ay - by) - (oy - ay) * (ax - bx)
        u_num = dx * (oy - ay) - dy * (ox - ax)
        t = -t_num / denom
        u = -u_num / denom
        if t > 0.0 and 0.0 <= u <= 1.0:
            return float(t)
        return float("inf")

    def _apply_lidar_noise(self, clean):
        """对激光雷达数据施加噪声 (使用当前回合的随机化参数).

        包含三种噪声源:
        1. 高斯距离噪声: 模拟测距精度误差
        2. 散斑噪声: 模拟边缘/角落处的反射干扰
        3. 随机丢点: 模拟镜面反射/黑色表面导致的信号丢失

        Args:
            clean: 无噪声的激光雷达数据

        Returns:
            np.ndarray: 带噪声的激光雷达数据
        """
        noisy = clean.copy()
        for b in range(self.N_SECTORS):
            r = clean[b]
            if r >= self.MAX_RANGE * 0.99:
                continue
            r += float(self._rng.normal(0.0, self._lidar_noise_std))
            if self._rng.random() < self._lidar_speckle_prob:
                r *= float(self._rng.uniform(
                    1.0 - self._lidar_speckle_scale, 1.0))
            if self._rng.random() < self._lidar_dropout_prob:
                r = self.MAX_RANGE
            noisy[b] = float(np.clip(r, 0.01, self.MAX_RANGE))
        return noisy

    # =================================================================
    # 生命体征传感器仿真 (FMCW 雷达, 保持核心逻辑不变)
    # =================================================================
    def _sim_life_sensor(self):
        """仿真 FMCW 毫米波雷达生命体征检测.

        模拟了真实雷达的关键特性:
        - 距离衰减: 距离越远信号越弱
        - 方位角衰减: 偏离雷达中心方向信号减弱
        - 废墟遮挡: 障碍物阻挡会降低信号质量
        - 多径虚警: 4% 概率产生虚假检测

        Returns:
            list[dict]: 检测结果列表, 每个包含 life_id, range, bearing, confidence
        """
        n_debris = len(self._obs_r)
        detections = []
        for i, lp in enumerate(self._life):
            if self._life_found[i]:
                continue
            rel = lp - self._pos
            dist = float(np.linalg.norm(rel))
            if dist > self.RADAR_MAX_RANGE:
                continue
            bearing = math.atan2(rel[1], rel[0]) - self._yaw
            bearing = math.atan2(math.sin(bearing), math.cos(bearing))
            if abs(bearing) > self.RADAR_FOV * 0.5:
                continue

            occlusion = self._line_occlusion(self._pos, lp, n_debris)
            range_score = max(0.0, 1.0 - dist / self.RADAR_MAX_RANGE)
            bearing_score = max(0.0, 1.0 - abs(bearing) / (self.RADAR_FOV * 0.5))
            snr = (0.55 * range_score + 0.35 * bearing_score
                   - 0.45 * occlusion
                   + float(self._rng.normal(0.0, 0.08)))
            confidence = float(np.clip(snr, 0.0, 1.0))
            measured_range = float(np.clip(
                dist + self._rng.normal(0.0, 0.08 + 0.04 * dist), 0.0, 9.9))

            if confidence > 0.20:
                detections.append({
                    "life_id": i, "range": measured_range,
                    "bearing": bearing, "confidence": confidence,
                    "occlusion": occlusion,
                })

        # 多径虚警
        if self._rng.random() < 0.04:
            detections.append({
                "life_id": None,
                "range": float(self._rng.uniform(0.8, self.RADAR_MAX_RANGE)),
                "bearing": float(self._rng.uniform(
                    -self.RADAR_FOV / 2, self.RADAR_FOV / 2)),
                "confidence": float(self._rng.uniform(0.18, 0.42)),
                "occlusion": 1.0,
            })
        return detections

    def _line_occlusion(self, start, end, n_debris=None):
        """计算两点之间的遮挡程度.

        检查所有障碍物是否阻挡了从 start 到 end 的视线.

        Args:
            start: 起点坐标
            end: 终点坐标
            n_debris: 障碍物数量

        Returns:
            float: 遮挡程度 [0, 1], 0 表示无遮挡
        """
        if n_debris is None:
            n_debris = len(self._obs_r)
        seg = end - start
        seg_len2 = float(np.dot(seg, seg))
        if seg_len2 <= 1e-9:
            return 0.0

        def cross2(a, b):
            return float(a[0] * b[1] - a[1] * b[0])

        def segment_intersects(p0, p1, q0, q1):
            r = p1 - p0
            s = q1 - q0
            denom = cross2(r, s)
            if abs(denom) < 1e-12:
                return False
            qp = q0 - p0
            t_val = cross2(qp, s) / denom
            u_val = cross2(qp, r) / denom
            return 0.0 < t_val < 1.0 and 0.0 <= u_val <= 1.0

        def point_in_poly(pt, verts):
            inside = False
            n = len(verts)
            x, y = float(pt[0]), float(pt[1])
            for idx in range(n):
                a = verts[idx]
                b = verts[(idx + 1) % n]
                yi, yj = float(a[1]), float(b[1])
                if (yi > y) != (yj > y):
                    x_cross = (float(b[0] - a[0]) * (y - yi)
                               / (yj - yi + 1e-12) + float(a[0]))
                    if x < x_cross:
                        inside = not inside
            return inside

        def point_segment_distance(pt, a, b):
            ab = b - a
            ab2 = float(np.dot(ab, ab))
            if ab2 < 1e-9:
                return float(np.linalg.norm(pt - a))
            t_val = np.clip(float(np.dot(pt - a, ab)) / ab2, 0.0, 1.0)
            return float(np.linalg.norm(pt - (a + t_val * ab)))

        blocked = 0.0
        for j in range(n_debris):
            nv = self._obs_nv[j]
            verts = [self._obs_c[j] + self._obs_v[j, k] for k in range(nv)]
            hit = False
            if point_in_poly(start, verts) or point_in_poly(end, verts):
                hit = True
            for k in range(nv):
                a = verts[k]
                b = verts[(k + 1) % nv]
                if segment_intersects(start, end, a, b):
                    hit = True
                    break
            if hit:
                blocked += 0.65
            else:
                near = False
                for k in range(nv):
                    a = verts[k]
                    b = verts[(k + 1) % nv]
                    distances = [
                        point_segment_distance(start, a, b),
                        point_segment_distance(end, a, b),
                        point_segment_distance(a, start, end),
                        point_segment_distance(b, start, end),
                    ]
                    if min(distances) < 0.35:
                        near = True
                        break
                if near:
                    blocked += 0.12
        return float(np.clip(blocked, 0.0, 1.0))

    # =================================================================
    # 观测构建 (迭代 2: sin/cos 编码, 迭代 6: 帧堆叠)
    # =================================================================
    def _get_single_obs(self):
        """构建单帧观测向量 (43 维).

        观测组成:
          [0:36]   激光雷达扇区距离 (归一化到 [0, 1])
          [36:38]  底盘速度 [vx/max_vx, wz/max_wz]
          [38]     目标距离 / 地图尺寸
          [39]     sin(目标相对航向)
          [40]     cos(目标相对航向)
          [41]     生命体征置信度
          [42]     生命体征距离 (归一化)

        Returns:
            np.ndarray: 43 维 float32 观测向量
        """
        lidar_raw = self._sim_lidar_clean()
        lidar = self._apply_lidar_noise(lidar_raw)
        gv = self._goal - self._pos
        gd = float(np.linalg.norm(gv))
        ga = math.atan2(gv[1], gv[0]) - self._yaw
        ga = math.atan2(math.sin(ga), math.cos(ga))

        obs = np.concatenate([
            lidar / self.MAX_RANGE,                          # [0:36] LiDAR
            [self._vx / self.MAX_VX,                         # [36] vx 归一化
             self._wz / self.MAX_WZ],                        # [37] wz 归一化
            [np.clip(gd / self.MAP_SIZE, 0, 1),              # [38] 目标距离
             math.sin(ga),                                   # [39] sin(航向)
             math.cos(ga)],                                  # [40] cos(航向)
            [self._last_life_conf,                           # [41] 生命置信度
             np.clip(self._last_life_range                   # [42] 生命距离
                     / self.RADAR_MAX_RANGE, 0, 1)],
        ]).astype(np.float32)
        return np.clip(obs, -1.0, 1.0)

    def _get_obs(self):
        """构建最终观测向量 (支持帧堆叠).

        如果启用帧堆叠, 返回 N_STACK 帧拼接后的向量;
        否则返回单帧向量.

        Returns:
            np.ndarray: 归一化后的观测向量
        """
        single_obs = self._get_single_obs()

        if not self._enable_frame_stack or self._obs_buffer is None:
            return single_obs

        # 将当前帧加入缓冲区
        self._obs_buffer.append(single_obs.copy())
        # 拼接所有帧 (时间维度: 旧 -> 新)
        stacked = np.concatenate(list(self._obs_buffer))
        return stacked.astype(np.float32)

    # =================================================================
    # 渲染 (pygame) -- 增加动态障碍物和幸存者运动的可视化
    # =================================================================
    def render(self):
        """渲染环境可视化 (使用 pygame).

        Returns:
            np.ndarray or None: rgb_array 模式下返回图像数组
        """
        if self.render_mode is None:
            return None
        try:
            import pygame
        except ImportError:
            return None

        size = 650
        scale = size / self.MAP_SIZE
        offset = size // 2
        if self._screen is None:
            pygame.init()
            self._screen = pygame.display.set_mode((size, size))
            pygame.display.set_caption("DisasterEnv v2.0")
            self._clock = pygame.time.Clock()

        self._screen.fill((20, 20, 20))

        # 绘制障碍物 (动态障碍物用不同颜色)
        n_debris = len(self._obs_r)
        for j in range(n_debris):
            nv = self._obs_nv[j]
            pts = []
            for k in range(nv):
                wx = self._obs_c[j, 0] + self._obs_v[j, k, 0]
                wy = self._obs_c[j, 1] + self._obs_v[j, k, 1]
                pts.append((int(wx * scale + offset),
                            int(-wy * scale + offset)))
            if nv >= 2:
                color = (220, 100, 30) if j < self._n_dynamic else (180, 80, 20)
                pygame.draw.polygon(self._screen, color, pts)

        # 绘制探索网格 (浅灰色覆盖)
        if self._explore_grid is not None:
            gs = self._explore_grid_size
            cell_px = max(1, int(self.EXPLORE_GRID_RES * scale))
            for ix in range(gs):
                for iy in range(gs):
                    if self._explore_grid[ix, iy]:
                        px = int(ix * cell_px)
                        py = int(iy * cell_px)
                        surf = pygame.Surface((cell_px, cell_px), pygame.SRCALPHA)
                        surf.fill((100, 100, 200, 30))
                        self._screen.blit(surf, (px, py))

        # 绘制幸存者
        for i, lp in enumerate(self._life):
            lx = int(lp[0] * scale + offset)
            ly = int(-lp[1] * scale + offset)
            color = (0, 200, 0) if self._life_found[i] else (255, 50, 50)
            pygame.draw.circle(self._screen, color, (lx, ly), 8)

        # 绘制目标
        gx = int(self._goal[0] * scale + offset)
        gy = int(-self._goal[1] * scale + offset)
        pygame.draw.circle(self._screen, (0, 200, 100), (gx, gy), 10)

        # 绘制机器人
        rx = int(self._pos[0] * scale + offset)
        ry = int(-self._pos[1] * scale + offset)
        pygame.draw.circle(self._screen, (60, 140, 240), (rx, ry), 10)
        ex = rx + int(18 * math.cos(self._yaw))
        ey = ry - int(18 * math.sin(self._yaw))
        pygame.draw.line(self._screen, (240, 200, 0), (rx, ry), (ex, ey), 3)

        pygame.display.flip()
        self._clock.tick(30)

        if self.render_mode == "rgb_array":
            return np.transpose(
                np.array(pygame.surfarray.pixels3d(self._screen)), (1, 0, 2))
        return None

    def close(self):
        """关闭渲染窗口."""
        if self._screen:
            import pygame
            pygame.quit()
            self._screen = None

    def get_observation_space_info(self):
        """返回观测空间的调试信息.

        Returns:
            dict: 包含 base_obs_dim, n_stack, total_obs_dim, obs_keys
        """
        return {
            "base_obs_dim": self._base_obs_dim,
            "n_stack": self.N_STACK if self._enable_frame_stack else 1,
            "total_obs_dim": self._base_obs_dim * (self.N_STACK if self._enable_frame_stack else 1),
            "obs_keys": [
                "lidar[0:36]", "vx[36]", "wz[37]",
                "goal_dist[38]", "sin_heading[39]", "cos_heading[40]",
                "life_conf[41]", "life_range[42]",
            ],
            "action_space": "Box(-1, 1, shape=(2,)) → [vx_cmd, wz_cmd]",
        }
