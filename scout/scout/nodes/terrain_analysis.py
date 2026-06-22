"""
terrain_analysis.py — 废墟地形分析节点

参考 PolarBear pb2025_sentry_nav 的 terrain_analysis 设计:
  1. 订阅 LiDAR 3D 点云 (/scout/points)
  2. RANSAC 地面平面分割
  3. 计算每个格栅的坡度和高度差
  4. 输出可通行性地图 (/scout/terrain_costmap)

硬件要求:
  - 3D LiDAR (推荐 Livox Mid-360, 也支持 2D LiDAR 降级模式)
  - 无 3D LiDAR 时, 使用 sim_swarm_node 的仿真数据降级运行

话题:
  订阅: /scout/points (PointCloud2) — 3D 点云
  订阅: /scout/scan (LaserScan) — 2D 降级模式
  发布: /scout/terrain_costmap (OccupancyGrid) — 地形代价地图
  发布: /scout/terrain_status (String) — 地形分析状态
"""
import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


class TerrainAnalyzer(Node):
    """废墟地形分析器.

    将传感器数据分类为三种地形:
      - 可通行 (cost=0): 坡度 < 15°, 高度差 < 0.15m
      - 警告区 (cost=50): 坡度 15-30°, 或高度差 0.15-0.3m
      - 不可通行 (cost=100): 坡度 > 30°, 或高度差 > 0.3m

    参考 PolarBear 的 terrain_analysis 设计:
      - 近场 (0-4m): 高分辨率分析, 决定即时避障
      - 远场 (4-10m): 低分辨率分析, 辅助路径规划
    """

    # 地图参数
    MAP_SIZE = 20.0       # 地图边长 m
    GRID_RES = 0.10       # 格栅分辨率 m
    MAX_HEIGHT_DIFF = 0.30  # 最大可通行高度差 m
    SLOPE_WARN = 15.0     # 警告坡度 (度)
    SLOPE_BLOCK = 30.0    # 不可通行坡度 (度)

    def __init__(self):
        super().__init__("terrain_analyzer")
        self.declare_parameter("use_3d_lidar", False)
        self.declare_parameter("max_range", 10.0)
        self.declare_parameter("robot_height", 0.30)  # 机器人离地高度 m

        self._use_3d = bool(self.get_parameter("use_3d_lidar").value)
        self._max_range = float(self.get_parameter("max_range").value)
        self._robot_h = float(self.get_parameter("robot_height").value)

        # 地形格栅: 0=未知, 1=可通行, 2=警告, 3=不可通行
        n = int(self.MAP_SIZE / self.GRID_RES)
        self._n = n
        self._terrain = np.zeros((n, n), np.int8)
        self._height_map = np.full((n, n), np.nan, np.float32)
        self._rx = 0.0
        self._ry = 0.0

        # QoS: 传感器数据用 BEST_EFFORT
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=5,
        )

        if self._use_3d:
            from sensor_msgs.msg import PointCloud2
            self.create_subscription(
                PointCloud2, "/scout/points", self._pointcloud_cb, qos)
            self.get_logger().info("Terrain analyzer: 3D LiDAR mode")
        else:
            # 降级: 用 2D LaserScan + 简化分析
            self.create_subscription(
                LaserScan, "/scout/scan", self._scan_cb, qos)
            self.get_logger().info("Terrain analyzer: 2D LiDAR fallback mode")

        from nav_msgs.msg import Odometry
        self.create_subscription(Odometry, "/scout/odom", self._odom_cb, 10)

        self._pub_map = self.create_publisher(
            OccupancyGrid, "/scout/terrain_costmap", 10)
        self._pub_status = self.create_publisher(
            String, "/scout/terrain_status", 10)

        self.create_timer(1.0, self._publish)
        self.get_logger().info(
            f"Terrain analyzer ready: {n}x{n} grid, res={self.GRID_RES}m")

    def _odom_cb(self, msg):
        self._rx = msg.pose.pose.position.x
        self._ry = msg.pose.pose.position.y

    def _scan_cb(self, scan: LaserScan):
        """2D LiDAR 降级模式: 将扫描数据标记为障碍物/可通行."""
        # 重置机器人附近的地形
        cx, cy = self._world2cell(self._rx, self._ry)
        clear_radius = int(0.5 / self.GRID_RES)
        for dx in range(-clear_radius, clear_radius + 1):
            for dy in range(-clear_radius, clear_radius + 1):
                nx, ny = cx + dx, cy + dy
                if self._in_bounds(nx, ny):
                    if dx * dx + dy * dy <= clear_radius * clear_radius:
                        self._terrain[ny, nx] = 1  # 可通行

        # 标记障碍物
        angle = scan.angle_min
        for r in scan.ranges:
            if scan.range_min < r < min(scan.range_max, self._max_range):
                ox = self._rx + r * math.cos(angle + 0.0)  # yaw=0 简化
                oy = self._ry + r * math.sin(angle)
                gx, gy = self._world2cell(ox, oy)
                if self._in_bounds(gx, gy):
                    # 障碍物周围标记为不可通行
                    obs_radius = int(0.25 / self.GRID_RES)
                    for ddx in range(-obs_radius, obs_radius + 1):
                        for ddy in range(-obs_radius, obs_radius + 1):
                            nx, ny = gx + ddx, gy + ddy
                            if self._in_bounds(nx, ny):
                                if ddx * ddx + ddy * ddy <= obs_radius * obs_radius:
                                    self._terrain[ny, nx] = 3
            angle += scan.angle_increment

    def _pointcloud_cb(self, msg):
        """3D LiDAR 模式: RANSAC 地面分割 + 坡度分析."""
        # 解析点云 (简化版, 实际应使用 sensor_msgs_py)
        # 这里提供框架, 具体解析取决于点云格式
        try:
            from sensor_msgs_py import point_cloud2
            points = list(point_cloud2.read_points(
                msg, field_names=("x", "y", "z"), skip_nans=True))
        except ImportError:
            self.get_logger().warn("sensor_msgs_py not available, using 2D fallback")
            return

        if len(points) < 100:
            return

        pts = np.array(points, dtype=np.float32)

        # RANSAC 地面平面拟合
        ground_mask = self._ransac_ground(pts, iterations=50, threshold=0.05)

        # 对地面点进行格栅分析
        ground_pts = pts[ground_mask]
        self._analyze_terrain_grid(ground_pts, pts[~ground_mask])

    def _ransac_ground(self, points, iterations=50, threshold=0.05):
        """RANSAC 地面平面分割.

        参考 PolarBear 的地面分割算法:
        - 随机采样 3 个点拟合平面
        - 计算所有点到平面的距离
        - 距离 < threshold 的点为地面

        Args:
            points: Nx3 numpy 数组
            iterations: RANSAC 迭代次数
            threshold: 地面点阈值 m

        Returns:
            np.ndarray: bool mask, True = 地面点
        """
        n = len(points)
        best_mask = np.zeros(n, dtype=bool)
        best_count = 0

        for _ in range(iterations):
            # 随机选 3 个点
            idx = np.random.choice(n, 3, replace=False)
            p0, p1, p2 = points[idx]

            # 计算法向量
            v1 = p1 - p0
            v2 = p2 - p0
            normal = np.cross(v1, v2)
            norm_len = np.linalg.norm(normal)
            if norm_len < 1e-6:
                continue
            normal /= norm_len

            # 确保法向量朝上 (z > 0)
            if normal[2] < 0:
                normal = -normal

            # 法向量与 z 轴夹角不能太大 (地面应该是相对平坦的)
            if abs(normal[2]) < 0.7:  # cos(45°) ≈ 0.7
                continue

            # 计算所有点到平面的距离
            d = -np.dot(normal, p0)
            distances = np.abs(np.dot(points, normal) + d)

            mask = distances < threshold
            count = np.sum(mask)

            if count > best_count:
                best_count = count
                best_mask = mask

        return best_mask

    def _analyze_terrain_grid(self, ground_pts, obstacle_pts):
        """对地面和障碍物点进行格栅分析."""
        # 重置高度图
        self._height_map[:] = np.nan

        # 填充地面高度
        for pt in ground_pts:
            gx, gy = self._world2cell(pt[0], pt[1])
            if self._in_bounds(gx, gy):
                old_h = self._height_map[gy, gx]
                if np.isnan(old_h) or pt[2] > old_h:
                    self._height_map[gy, gx] = pt[2]

        # 计算坡度和高度差
        for y in range(1, self._n - 1):
            for x in range(1, self._n - 1):
                h = self._height_map[y, x]
                if np.isnan(h):
                    self._terrain[y, x] = 0  # 未知
                    continue

                # 计算与相邻格栅的最大高度差
                max_diff = 0.0
                for dy, dx in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                    nh = self._height_map[y + dy, x + dx]
                    if not np.isnan(nh):
                        max_diff = max(max_diff, abs(h - nh))

                # 计算坡度 (用高度差近似)
                slope = math.degrees(math.atan2(max_diff, self.GRID_RES))

                if slope > self.SLOPE_BLOCK or max_diff > self.MAX_HEIGHT_DIFF:
                    self._terrain[y, x] = 3  # 不可通行
                elif slope > self.SLOPE_WARN or max_diff > 0.15:
                    self._terrain[y, x] = 2  # 警告
                else:
                    self._terrain[y, x] = 1  # 可通行

    def _publish(self):
        """发布地形代价地图 (Nav2 兼容格式)."""
        grid = OccupancyGrid()
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = "map"
        grid.info.resolution = self.GRID_RES
        grid.info.width = self._n
        grid.info.height = self._n
        grid.info.origin.position.x = -self.MAP_SIZE / 2.0
        grid.info.origin.position.y = -self.MAP_SIZE / 2.0
        grid.info.origin.orientation.w = 1.0

        # 转换为 Nav2 costmap 格式: 0=free, 50=lethal, 100=inscribed
        costmap = np.zeros((self._n, self._n), np.int8)
        costmap[self._terrain == 2] = 50    # 警告
        costmap[self._terrain == 3] = 100   # 不可通行
        grid.data = costmap.ravel().tolist()
        self._pub_map.publish(grid)

        # 发布状态
        n_free = int(np.sum(self._terrain == 1))
        n_warn = int(np.sum(self._terrain == 2))
        n_block = int(np.sum(self._terrain == 3))
        n_total = self._n * self._n
        msg = String()
        msg.data = (f"free={n_free}({100*n_free//n_total}%) "
                    f"warn={n_warn}({100*n_warn//n_total}%) "
                    f"block={n_block}({100*n_block//n_total}%)")
        self._pub_status.publish(msg)

    def _world2cell(self, x, y):
        gx = int((x + self.MAP_SIZE / 2.0) / self.GRID_RES)
        gy = int((y + self.MAP_SIZE / 2.0) / self.GRID_RES)
        return gx, gy

    def _in_bounds(self, cx, cy):
        return 0 <= cx < self._n and 0 <= cy < self._n


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(TerrainAnalyzer())
    rclpy.shutdown()
