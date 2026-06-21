# 操作手册

> 详细操作说明请参阅项目根目录的 **[README.md](../README.md)**，其中包含从零搭建、仿真运行、实车部署、Web 地面站、RL 训练的完整教程。

## 快速参考

### 启动命令速查

| 场景 | 命令 |
|------|------|
| 轻量 Demo | `ros2 launch swarm_bringup demo_human_follow.launch.py` |
| Gazebo 仿真 | `ros2 launch swarm_bringup sim.launch.py` |
| 实车完整 | `ros2 launch swarm_bringup swarm.launch.py` |
| 仅 Scout | `ros2 launch scout scout.launch.py` |
| Web 地面站 | `ros2 run ground_station web_dashboard` |

### 话题速查

| 话题 | 类型 | 说明 |
|------|------|------|
| `/gs/command` | String | 地面站指令入口 |
| `/swarm/status` | String | 全局状态 JSON |
| `/scout/life_detections` | Float32MultiArray | 生命体征 [range, breath, heart, conf] |
| `/carrier/supply_status` | String | 物资库存状态 |
| `/specialist/arm_status` | String | 机械臂状态 |
| `/scout/hw_status` | String | Scout 硬件状态 (含 OFFLINE 检测) |

### 指令格式

```
robot:action[:args...]

示例:
scout:emergency          # Scout 紧急停止
carrier:supply:water:0   # Carrier 投送饮用水到槽位 0
specialist:arm_task:inspect  # Specialist 机械臂检查动作
specialist:led:3:255:255:0:0  # Specialist LED SOS 模式
```
