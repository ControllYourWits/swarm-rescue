# 三机协同救援机器人集群系统 (Swarm Rescue)

> 面向地震/塌方废墟场景的三机协同救援系统：Scout 探测导航 + Carrier 补给运输 + Specialist 作业处置

![ROS2](https://img.shields.io/badge/ROS2-Jazzy/Humble-blue)
![STM32](https://img.shields.io/badge/STM32-FreeRTOS-green)
![Python](https://img.shields.io/badge/Python-3.10+-yellow)
![License](https://img.shields.io/badge/License-MIT-orange)

---

## 目录

- [系统概览](#系统概览)
- [硬件架构](#硬件架构)
- [软件架构](#软件架构)
- [环境搭建（从零开始）](#环境搭建从零开始)
  - [1. WSL2/Ubuntu 基础环境](#1-wsl2ubuntu-基础环境)
  - [2. ROS2 安装](#2-ros2-安装)
  - [3. 克隆项目](#3-克隆项目)
  - [4. 安装 Python 依赖](#4-安装-python-依赖)
  - [5. 构建 ROS2 工作空间](#5-构建-ros2-工作空间)
  - [6. STM32 固件编译（可选）](#6-stm32-固件编译可选)
- [运行指南](#运行指南)
  - [仿真模式（无硬件）](#仿真模式无硬件)
  - [实车模式](#实车模式)
  - [仅 Scout 调试](#仅-scout-调试)
- [地面站使用](#地面站使用)
  - [命令行控制](#命令行控制)
  - [Web Dashboard](#web-dashboard)
- [RL 训练](#rl-训练)
  - [训练 Scout 策略](#训练-scout-策略)
  - [评估模型](#评估模型)
  - [ONNX 导出与验证](#onnx-导出与验证)
- [通信协议](#通信协议)
- [项目结构](#项目结构)
- [关键参数速查](#关键参数速查)
- [故障排查](#故障排查)
- [开发指南](#开发指南)

---

## 系统概览

三台机器人各司其职，在废墟环境中协同完成搜索、补给和救援任务：

| 机器人 | 代号 | 核心功能 | 计算平台 |
|--------|------|----------|----------|
| 探测先遣机 | **Scout** | RL 自主导航 + 毫米波生命探测 + SLAM 建图 | RK3588 + STM32F407 |
| 补给运输机 | **Carrier** | 4G/5G 中继 + 跟随导航 + 物资投送 (4 槽位) | 树莓派 4B + STM32F407 |
| 作业处置机 | **Specialist** | 热成像检测 + 4-DOF 机械臂 + LED 补光 | RK3588 + STM32F407 |

### 工作流程

```
Scout 入场探索 → RL 导航避障 → 毫米波雷达扫描生命体征
        ↓ 发现幸存者
地面站自动调度 → Carrier 投送物资 → Specialist 机械臂清理/检查
        ↓ 4G/5G 中继
远程指挥中心实时监控 (Web Dashboard)
```

---

## 硬件架构

```
                    ┌──────────────────────┐
                    │   远程指挥中心 PC     │
                    │  Web Dashboard :5000  │
                    └──────────┬───────────┘
                               │ 4G/5G
                    ┌──────────┴───────────┐
                    │  GL.iNet GL-X3000    │
                    │  (Carrier 车载路由)   │
                    └──────────┬───────────┘
                               │ WiFi Mesh
            ┌──────────────────┼──────────────────┐
            │                  │                  │
   ┌────────┴───────┐ ┌───────┴────────┐ ┌───────┴────────┐
   │     Scout      │ │    Carrier     │ │   Specialist   │
   │  RK3588+STM32  │ │ RPi4B+STM32   │ │  RK3588+STM32  │
   │  激光雷达×1    │ │ 物资舱×4      │ │  热成像相机    │
   │  毫米波雷达×1  │ │ UWB Tag       │ │  机械臂 4-DOF  │
   │  BMI088 IMU    │ │ 电池 6S       │ │  LED 补光灯    │
   │  M3508×4 轮   │ │ M3508×4 轮    │ │  M3508×4 轮    │
   └────────────────┘ └────────────────┘ └────────────────┘
         │  CAN 1Mbps          │               │
         │  UART 921600        │               │
         └─── STM32F407 ───────┘───────────────┘
              (FreeRTOS 1kHz 底盘 PID)
```

---

## 软件架构

### ROS2 节点拓扑

```
┌─ Scout ──────────────────────────────────────┐
│  serial_bridge    ←→ STM32 (odom/imu/cmd_vel)│
│  lidar_proc       → /scout/obstacle_distances │
│  rl_nav_node      ← obstacle + goal → cmd_vel│
│  radar_processor  → /scout/life_detections    │
│  life_map_node    → /scout/life_map + goal    │
│  nav2_goal_bridge → Nav2 (可选)               │
└──────────────────────────────────────────────┘

┌─ Carrier ────────────────────────────────────┐
│  serial_bridge    ←→ STM32 (odom/battery)     │
│  follow_navigator → /carrier/cmd_vel          │
│  supply_manager   → /carrier/supply_cmd       │
│  relay_manager    → /carrier/network_status   │
│  lora_bridge      → LoRa 灾备链路             │
└──────────────────────────────────────────────┘

┌─ Specialist ─────────────────────────────────┐
│  serial_bridge    ←→ STM32 (odom/arm/led)     │
│  arm_planner      → /specialist/arm_cmd       │
│  thermal_node     → /specialist/thermal_*     │
│  follow_navigator → /specialist/cmd_vel       │
└──────────────────────────────────────────────┘

┌─ 地面站 ─────────────────────────────────────┐
│  ground_station_node  全局监控 + 自动调度     │
│  web_dashboard        Flask Web 实时仪表盘    │
└──────────────────────────────────────────────┘
```

### 通信链路

```
互联网/4G ← GL.iNet GL-X3000 (Carrier 车载)
               ↓ WiFi Mesh (同一 ROS_DOMAIN_ID=42)
Scout ←──────────────→ Carrier ←──────→ Specialist

备份: LoRa (1~2km, 低带宽心跳, 网络断开自动激活)
定位: UWB (三机相对定位 ±10cm)
协议: 自定义二进制帧 [0xAA][LEN][MSG_ID][PAYLOAD][CRC8][0x55]
```

---

## 环境搭建（从零开始）

### 前置要求

| 项目 | 最低配置 | 推荐配置 |
|------|----------|----------|
| 操作系统 | Ubuntu 22.04 / WSL2 | Ubuntu 24.04 |
| ROS2 版本 | Humble | Jazzy |
| Python | 3.10 | 3.12 |
| GPU (训练用) | 无 (CPU 可训练) | NVIDIA GTX 1060+ |
| 内存 | 4 GB | 8 GB+ |
| 磁盘 | 10 GB | 20 GB+ |

### 1. WSL2/Ubuntu 基础环境

**Windows 用户推荐使用 WSL2**：

```powershell
# Windows PowerShell (管理员)
wsl --install -d Ubuntu-24.04
# 重启后设置用户名密码
```

进入 WSL 后安装基础工具：

```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y git curl wget python3-pip python3-venv \
    build-essential cmake libserial-dev
```

### 2. ROS2 安装

**Ubuntu 24.04 → ROS2 Jazzy**：

```bash
# 添加 ROS2 源
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) \
    signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
    sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 安装 ROS2 (桌面版含 RViz2/Gazebo)
sudo apt update
sudo apt install -y ros-jazzy-desktop

# 安装额外依赖
sudo apt install -y \
    ros-jazzy-nav2-bringup \
    ros-jazzy-slam-toolbox \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-tf2-ros \
    ros-jazzy-xacro \
    ros-jazzy-rosbridge-suite \
    python3-colcon-common-extensions \
    python3-rosdep

# 环境变量 (添加到 ~/.bashrc)
echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc
source ~/.bashrc
```

> **Ubuntu 22.04 用户**：将 `jazzy` 替换为 `humble`。

### 3. 克隆项目

```bash
# 克隆到用户目录
git clone <仓库地址> ~/swarm_rescue
cd ~/swarm_rescue

# 或者从 Windows 拷贝 (WSL2 自动挂载)
# cp -r /mnt/e/swarm_rescue_final/swarm_rescue ~/swarm_rescue
```

### 4. 安装 Python 依赖

```bash
# ROS2 节点依赖
pip3 install pyserial requests flask flask-socketio numpy

# RL 训练依赖 (仅在训练 PC 上安装)
cd ~/swarm_rescue/rl_training
pip3 install -r requirements.txt
```

`requirements.txt` 内容：
```
torch>=2.0.0
stable-baselines3[extra]>=2.2.0
gymnasium>=0.29.0
numpy>=1.24.0
onnxruntime>=1.16.0
tensorboard>=2.13.0
pygame>=2.4.0
```

### 5. 构建 ROS2 工作空间

```bash
# 创建工作空间并链接源码
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# 符号链接各个 ROS2 包 (推荐, 修改源码后无需重新拷贝)
ln -sf ~/swarm_rescue/scout/ros2_pkg         scout
ln -sf ~/swarm_rescue/carrier/ros2_pkg       carrier
ln -sf ~/swarm_rescue/specialist/ros2_pkg    specialist
ln -sf ~/swarm_rescue/swarm_msgs             swarm_msgs
ln -sf ~/swarm_rescue/swarm_bringup          swarm_bringup
ln -sf ~/swarm_rescue/ground_station         ground_station
ln -sf ~/swarm_rescue/shared                 shared

# 初始化 rosdep
sudo rosdep init 2>/dev/null || true
rosdep update
rosdep install --from-paths . --ignore-src -r -y

# 构建 (首次约 30 秒)
cd ~/ros2_ws
colcon build --symlink-install

# 激活环境 (每次打开新终端都需要)
source install/setup.bash

# 验证安装
ros2 pkg list | grep -E "scout|carrier|specialist|swarm"
# 应输出: carrier, ground_station, scout, specialist, swarm_bringup, swarm_msgs
```

> **关键提示**：构建后如果节点找不到，运行以下命令修复 symlink：
> ```bash
> cd ~/ros2_ws
> mkdir -p install/swarm_bringup/lib/swarm_bringup
> for exe in install/swarm_bringup/bin/*; do
>     ln -sf $(pwd)/$exe install/swarm_bringup/lib/swarm_bringup/$(basename $exe)
> done
> mkdir -p install/carrier/lib/carrier
> for exe in install/carrier/bin/*; do
>     ln -sf $(pwd)/$exe install/carrier/lib/carrier/$(basename $exe)
> done
> ```

### 6. STM32 固件编译（可选）

如需烧录底层固件到 STM32F407：

```bash
# 安装 ARM 工具链
sudo apt install -y gcc-arm-none-eabi

# 编译 Scout 固件
cd ~/swarm_rescue/scout/stm32_fw
make -j$(nproc)
# 输出: build/scout_fw.hex

# 烧录 (需要 ST-Link 或 J-Link)
make flash

# Carrier / Specialist 同理
cd ~/swarm_rescue/carrier/stm32_fw && make -j$(nproc)
cd ~/swarm_rescue/specialist/stm32_fw && make -j$(nproc)
```

---

## 运行指南

### 仿真模式（无硬件）

最简单的验证方式，不需要任何硬件设备：

```bash
source ~/ros2_ws/install/setup.bash

# 启动 Gazebo 3D 仿真 (推荐有 GPU)
ros2 launch swarm_bringup sim.launch.py

# 或轻量级无 Gazebo 仿真 (任何机器都能跑)
ros2 launch swarm_bringup demo_human_follow.launch.py
```

**仿真模式说明**：
- `sim_swarm_node` 仿真三机物理运动，发布 odom/laser/life 数据
- `lidar_proc` 和 `life_map_node` 自动以 `use_sim:=true` 启动，避免话题冲突
- Carrier 自动跟随 Scout（或跟随虚拟人类）
- 地面站自动监控并调度

### 实车模式

连接真实硬件后：

```bash
source ~/ros2_ws/install/setup.bash

# 完整三机启动 (Scout 先启动, Carrier/Specialist 延时 3s)
ros2 launch swarm_bringup swarm.launch.py

# 指定串口
ros2 launch swarm_bringup swarm.launch.py \
    scout_port:=/dev/ttyS3 \
    carrier_port:=/dev/ttyS4 \
    specialist_port:=/dev/ttyS5

# 仅启动 Scout
ros2 launch swarm_bringup swarm.launch.py \
    enable_carrier:=false enable_specialist:=false
```

### 仅 Scout 调试

```bash
# 启动单个 Scout (含 SLAM + RL 导航)
ros2 launch scout scout.launch.py

# 使用 Nav2 替代 RL 导航
ros2 launch scout scout.launch.py use_nav2:=true
```

---

## 地面站使用

### 命令行控制

通过 `/gs/command` 话题发送指令，格式为 `robot:action[:args...]`：

```bash
# ── 运动控制 ──
ros2 topic pub /gs/command std_msgs/msg/String '{data: "scout:stop"}'
ros2 topic pub /gs/command std_msgs/msg/String '{data: "scout:normal"}'
ros2 topic pub /gs/command std_msgs/msg/String '{data: "scout:emergency"}'
ros2 topic pub /gs/command std_msgs/msg/String '{data: "carrier:follow"}'
ros2 topic pub /gs/command std_msgs/msg/String '{data: "specialist:emergency"}'

# ── 物资投送 ──
ros2 topic pub /gs/command std_msgs/msg/String '{data: "carrier:supply:water:0"}'
ros2 topic pub /gs/command std_msgs/msg/String '{data: "carrier:supply:firstaid:1"}'
ros2 topic pub /gs/command std_msgs/msg/String '{data: "carrier:supply:battery:2"}'
ros2 topic pub /gs/command std_msgs/msg/String '{data: "carrier:supply:rope:3"}'

# ── 机械臂 ──
ros2 topic pub /gs/command std_msgs/msg/String '{data: "specialist:arm_task:home"}'
ros2 topic pub /gs/command std_msgs/msg/String '{data: "specialist:arm_task:push"}'
ros2 topic pub /gs/command std_msgs/msg/String '{data: "specialist:arm_task:clear"}'
ros2 topic pub /gs/command std_msgs/msg/String '{data: "specialist:arm_task:inspect"}'
ros2 topic pub /gs/command std_msgs/msg/String '{data: "specialist:arm_task:deliver"}'

# ── LED 控制 ──
ros2 topic pub /gs/command std_msgs/msg/String '{data: "specialist:led:1:255:255:0:0"}'
# 格式: mode:brightness:r:g:b  mode: 0=关 1=常亮 2=频闪 3=SOS

# ── RL 控制 ──
ros2 topic pub /gs/command std_msgs/msg/String '{data: "scout:rl_enable:true"}'
ros2 topic pub /gs/command std_msgs/msg/String '{data: "scout:rl_enable:false"}'

# ── 监控 ──
ros2 topic echo /swarm/status           # 全局状态 JSON
ros2 topic echo /scout/life_detections  # 生命体征检测
ros2 topic echo /scout/rl_status        # RL 导航状态
ros2 topic echo /carrier/battery        # Carrier 电量
ros2 topic echo /specialist/arm_status  # 机械臂状态
```

### Web Dashboard

基于 Flask + WebSocket 的实时仪表盘：

```bash
source ~/ros2_ws/install/setup.bash

# 启动 Web 地面站
ros2 run ground_station web_dashboard
```

浏览器打开 **http://localhost:5000** 即可看到：
- 三机实时位置和状态
- 生命体征检测置信度
- 物资库存状态
- 一键指令按钮（紧急停止、投送、检查）
- 任务日志时间线

> **远程访问**：在同一局域网内，用 `http://<机器人IP>:5000` 访问。

---

## RL 训练

### 训练 Scout 策略

在**独立 PC**（有 GPU 更快，CPU 也能训练）上进行：

```bash
cd ~/swarm_rescue/rl_training

# 快速测试 (50k 步, ~2 分钟)
python train_scout.py --fast

# 标准训练 (5M 步, GPU 约 2 小时, CPU 约 8 小时)
python train_scout.py --n_envs 8

# 自定义参数
python train_scout.py --timesteps 10000000 --n_envs 16 --seed 42

# 禁用课程学习 (直接用满难度)
python train_scout.py --no-curriculum

# 从检查点恢复
python train_scout.py --resume checkpoints/scout_ppo_2000000_steps.zip
```

**训练输出**：
```
checkpoints/
├── best/best_model.zip      # 最优模型
├── scout_ppo_*_steps.zip    # 定期检查点
├── scout_final.zip          # 最终模型
├── scout_policy.onnx        # ONNX 推理模型
└── scout_norm.npz           # 归一化参数
```

**TensorBoard 监控**：

```bash
# 新终端
tensorboard --logdir ~/swarm_rescue/rl_training/logs --port 6006
# 浏览器打开 http://localhost:6006
```

关键指标：
| 指标 | 含义 | 目标值 |
|------|------|--------|
| `custom/success_rate` | 目标达成率 | > 0.8 |
| `custom/collision_rate` | 碰撞率 | < 0.1 |
| `custom/life_found_rate` | 幸存者发现率 | > 0.6 |
| `curriculum/level` | 课程等级 | 渐进到 1.0 |

### 评估模型

```bash
cd ~/swarm_rescue/rl_training

# SB3 模型评估
python eval_scout.py --model checkpoints/best/best_model.zip --episodes 50

# ONNX 模型评估
python eval_scout.py --onnx checkpoints/scout_policy.onnx --episodes 50

# 高难度评估 + 可视化
python eval_scout.py --model checkpoints/scout_final.zip \
    --episodes 20 --curriculum 1.0 --render

# 输出到 CSV
python eval_scout.py --model checkpoints/best/best_model.zip \
    --episodes 100 --output results.csv
```

输出示例：
```
==================================================
  Evaluation Results (50 episodes)
==================================================
  Success rate:    82.0% (41/50)
  Collision rate:  8.0% (4/50)
  Boundary rate:   4.0% (2/50)
  Timeout rate:    6.0% (3/50)
  Life found rate: 68.0%
  Avg reward:      156.3 ± 45.2
  Avg steps:       487 ± 213
==================================================
```

### ONNX 导出与验证

```bash
# 验证 ONNX 模型精度 (对比 PyTorch)
python verify_onnx.py \
    --model checkpoints/scout_final.zip \
    --onnx checkpoints/scout_policy.onnx

# 仅验证 ONNX 模型 (无 PyTorch 依赖)
python verify_onnx.py --onnx checkpoints/scout_policy.onnx --samples 500
```

验证内容：
1. **基础推理** — 模型能正常加载和推理
2. **输出范围** — 动作在 [-1, 1] 内
3. **确定性** — 相同输入产生相同输出
4. **批量测试** — 200 个随机输入全部有限且在范围内

部署到 RK3588：
```bash
# 拷贝模型到机器人
scp checkpoints/scout_policy.onnx robot@scout:/home/robot/models/
scp checkpoints/scout_norm.npz robot@scout:/home/robot/models/

# 在 RK3588 上验证
python3 verify_onnx.py --onnx /home/robot/models/scout_policy.onnx \
    --norm /home/robot/models/scout_norm.npz
```

---

## 通信协议

### 帧格式

```
[0xAA] [LEN] [MSG_ID] [PAYLOAD...] [CRC8] [0x55]
  1B     1B     1B       LEN-1 B      1B      1B
```

- `LEN` = PAYLOAD 长度 + 1 (包含 MSG_ID)
- `CRC8` = 对 `LEN + MSG_ID + PAYLOAD` 的 CRC-8 校验 (多项式 0x07)

### 消息定义

| MSG_ID | 方向 | 名称 | 大小 | 说明 |
|--------|------|------|------|------|
| 0x01 | ↑上行 | ODOM | 24B | 里程计 (vx,vy,wz,px,py,yaw) |
| 0x02 | ↑上行 | IMU | 36B | 加速度 + 陀螺仪 + 欧拉角 |
| 0x03 | ↑上行 | STATUS | 8B | 机器人状态 (模式/电机/电池) |
| 0x04 | ↑上行 | BATTERY | 12B | 电池电压/电流/百分比 |
| 0x10 | ↓下行 | CMD_VEL | 16B | 速度指令 (vx,vy,wz,mode) |
| 0x11 | ↓下行 | SET_MODE | 1B | 模式切换 (STOP/NORMAL/RC/EMERGENCY) |
| 0x12 | ↓下行 | ARM_CMD | 20B | 机械臂 4 关节角度 + 夹爪 |
| 0x13 | ↓下行 | SUPPLY_CMD | 4B | 物资投送 (action, slot) |
| 0x14 | ↓下行 | LED_CMD | 8B | LED 控制 (mode,brightness,r,g,b) |
| 0x1F | ↓下行 | HEARTBEAT | 1B | 心跳保活 |

### 底盘模式

| 值 | 模式 | 说明 |
|----|------|------|
| 0x00 | STOP | 停止 |
| 0x01 | NORMAL | 正常自主模式 |
| 0x02 | FOLLOW | 跟随模式 (Carrier) |
| 0x03 | RC | 遥控接管 |
| 0xFF | EMERGENCY | 紧急停止 |

---

## 项目结构

```
swarm_rescue/
├── README.md                          # 本文档
├── .gitignore                         # Git 忽略规则
├── docker-compose.yml                 # Docker 部署 (可选)
│
├── shared/                            # 跨模块共用代码
│   ├── protocol/
│   │   ├── swarm_protocol.h           #   通信协议 C 头文件 (STM32 端)
│   │   └── swarm_protocol.py          #   通信协议 Python 版 (ROS2 端)
│   └── qos_profiles.py                #   DDS QoS 配置 (控制/传感器)
│
├── scout/                             # 探测先遣机
│   ├── stm32_fw/                      #   STM32 FreeRTOS 固件
│   │   ├── Algorithm/                 #     PID / 运动学 / Mahony AHRS
│   │   ├── BSP/                       #     CAN / UART 底层驱动
│   │   ├── Driver/                    #     M3508 电机 / BMI088 IMU
│   │   ├── Module/                    #     底盘控制 / 通信模块
│   │   └── Core/                      #     FreeRTOS 主入口 + SBUS 遥控
│   └── ros2_pkg/                      #   ROS2 节点
│       ├── scout/nodes/
│       │   ├── serial_bridge.py       #     STM32 串口桥 + 心跳看门狗
│       │   ├── lidar_proc.py          #     激光雷达扇区化
│       │   ├── rl_nav_node.py         #     RL ONNX 推理导航
│       │   ├── radar_processor.py     #     毫米波生命体征处理
│       │   ├── life_map_node.py       #     生命概率栅格地图
│       │   └── nav2_goal_bridge.py    #     Nav2 目标桥接 (可选)
│       ├── scout/launch/
│       │   └── scout.launch.py        #     Scout 启动文件
│       └── scout/config/
│           └── nav2_params.yaml       #     Nav2 参数
│
├── carrier/                           # 补给运输机
│   ├── stm32_fw/                      #   STM32 固件 (底盘+物资舵机)
│   │   └── Module/
│   │       ├── carrier_ctrl.c         #     底盘+电池 ADC+舵机 PWM
│   │       ├── supply.c               #     物资投送状态机 (4 槽位)
│   │       └── comm.c                 #     通信帧收发
│   └── ros2_pkg/
│       └── carrier/nodes/
│           ├── serial_bridge.py       #     串口桥 + 心跳看门狗
│           ├── follow_navigator.py    #     跟随控制器 (direct/Nav2)
│           ├── supply_manager.py      #     物资管理 + 自动投送
│           ├── human_simulator.py     #     虚拟人类仿真 (demo 用)
│           ├── relay_manager.py       #     4G/5G 路由监控
│           └── lora_bridge_node.py    #     LoRa 灾备链路
│
├── specialist/                        # 作业处置机
│   ├── stm32_fw/                      #   STM32 固件
│   │   └── Module/
│   │       ├── arm.c                  #     4-DOF 机械臂插值控制
│   │       ├── led.c                  #     RGB LED (常亮/频闪/SOS)
│   │       └── spec_ctrl.c            #     底盘控制封装
│   └── ros2_pkg/
│       └── specialist/nodes/
│           ├── serial_bridge.py       #     串口桥 + 心跳看门狗
│           ├── arm_planner.py         #     机械臂动作序列
│           └── thermal_node.py        #     热成像处理 (仿真/真实)
│
├── ground_station/                    # 地面指挥站
│   └── ground_station/
│       ├── ground_station_node.py     #   全局监控 + 自动调度
│       ├── web_dashboard.py           #   Flask Web 实时仪表盘
│       └── templates/
│           └── dashboard.html         #   Web 前端页面
│
├── swarm_msgs/                        # 自定义 ROS2 消息
│   └── msg/
│       ├── LifeSign.msg               #   生命体征 (range, freq, conf)
│       ├── SwarmStatus.msg            #   集群状态
│       └── ArmTask.msg                #   机械臂任务
│
├── swarm_bringup/                     # 统一启动与仿真
│   ├── launch/
│   │   ├── swarm.launch.py            #   完整三机实车启动
│   │   ├── sim.launch.py              #   Gazebo 仿真启动
│   │   └── demo_human_follow.launch.py #  轻量 demo (无 Gazebo)
│   ├── urdf/                          #   三机 URDF 模型
│   ├── config/                        #   SLAM/CycloneDDS/RViz 配置
│   └── worlds/                        #   Gazebo 废墟世界文件
│
├── rl_training/                       # RL 强化学习
│   ├── disaster_env.py                #   废墟环境 (Gymnasium, 13 项改进)
│   ├── train_scout.py                 #   PPO 训练脚本 (20 项改进)
│   ├── eval_scout.py                  #   模型评估 (输出 CSV 报告)
│   ├── verify_onnx.py                 #   ONNX 导出精度验证
│   └── requirements.txt               #   Python 依赖
│
├── test/                              # 单元测试
│   ├── test_protocol.py               #   通信协议完整测试
│   ├── test_chassis_kinematics.py     #   底盘运动学+PID+Mahony
│   ├── test_disaster_env.py           #   废墟环境接口测试
│   ├── test_robot_roles.py            #   机器人角色测试
│   └── test_swarm_collaboration.py    #   协同逻辑测试
│
└── docs/                              # 项目文档
    ├── code_review.md                 #   架构审查建议
    ├── operation_manual.md            #   操作手册
    └── 三机协同救援机器人集群系统.pdf  #   系统设计文档
```

---

## 关键参数速查

### 物理参数

| 参数 | 值 | 说明 |
|------|----|------|
| 最大线速度 | 0.6 m/s | 废墟中限速 |
| 最大角速度 | 2.0 rad/s | 原地旋转 |
| 轮半径 | 0.0765 m | M3508 麦轮 |
| 轴距 (LXY) | 0.2025 m | 麦轮几何中心到轮心距离 |

### 传感器参数

| 参数 | 值 | 说明 |
|------|----|------|
| 激光雷达最大量程 | 6.0 m | 36 扇区 |
| 毫米波雷达量程 | 5.0 m | AWR1642 |
| 雷达视场角 | 130° | 生命探测覆盖 |
| 热成像分辨率 | 120×160 | 模拟/真实 |

### 通信参数

| 参数 | 值 | 说明 |
|------|----|------|
| CAN 波特率 | 1 Mbps | DJI C620 电调 |
| UART 波特率 | 921600 | STM32↔RK3588 |
| 心跳周期 | 1 s | ROS→STM32 |
| 指令超时 | 0.5-0.8 s | 无指令自动 STOP |
| STM32 掉线检测 | 3 s | 看门狗发布 OFFLINE |
| LoRa 波特率 | 115200 | 灾备链路 |

### RL 训练参数

| 参数 | 值 | 说明 |
|------|----|------|
| 观测空间 | 43 维 | 36 LiDAR + 2 速度 + 3 目标 + 2 生命 |
| 动作空间 | 2 维 | [vx_cmd, wz_cmd] ∈ [-1, 1] |
| 学习率 | 3e-4 → 1e-5 | 线性退火 |
| 网络 | [512, 256, 128] | pi + vf 分离 |
| n_steps | 4096 | rollout 缓冲区 |
| 碰撞距离 | 0.28 m | 终止条件 |
| 目标半径 | 0.35 m | 到达判定 |
| 地图大小 | 15×15 m | 默认仿真地图 |

---

## 故障排查

### ROS2 构建失败

```bash
# 清理重建
cd ~/ros2_ws
rm -rf build/ install/ log/
colcon build --symlink-install

# 如果 swarm_msgs 编译失败，先单独编译
colcon build --packages-select swarm_msgs
colcon build --symlink-install
```

### 节点找不到

```bash
# 检查包是否注册
ros2 pkg list | grep scout

# 检查可执行文件
ros2 pkg executables scout

# 如果找不到，检查 setup.py 中的 entry_points
```

### 串口权限问题

```bash
# 将用户加入 dialout 组
sudo usermod -aG dialout $USER
# 重新登录生效

# 或临时赋权
sudo chmod 666 /dev/ttyS3
```

### sim.launch.py 双发布者冲突

如果看到 obstacle_distances 或 radar_trigger 有多个发布者，确认 `use_sim` 参数已传入：

```bash
# 检查当前发布者
ros2 topic info /scout/obstacle_distances
ros2 topic info /scout/radar_trigger

# 应该只有 sim_swarm_node 一个发布者
```

### STM32 串口无数据

1. 检查接线：TX→RX, RX→TX, GND 共地
2. 检查波特率：必须 921600
3. 检查固件是否烧录成功（LED 闪烁）
4. 查看日志：`ros2 topic echo /scout/hw_status`，应显示 mode/battery 信息

### RL 训练不收敛

```bash
# 检查 TensorBoard 曲线
tensorboard --logdir rl_training/logs

# 降低难度重训
python train_scout.py --no-curriculum --timesteps 2000000

# 验证环境
cd rl_training && python -c "
from disaster_env import DisasterEnv
env = DisasterEnv(enable_frame_stack=False)
print(env.get_observation_space_info())
"
```

---

## 开发指南

### 添加新节点

1. 在对应包的 `nodes/` 目录创建 `.py` 文件
2. 在 `setup.py` 的 `entry_points` 注册
3. 在 launch 文件中添加节点

```python
# 示例: scout/ros2_pkg/setup.py
entry_points={"console_scripts": [
    "serial_bridge = scout.nodes.serial_bridge:main",
    "lidar_proc = scout.nodes.lidar_proc:main",
    "my_new_node = scout.nodes.my_new_node:main",  # 新增
]}
```

### 话题命名规范

| 前缀 | 含义 | 示例 |
|------|------|------|
| `/scout/` | Scout 专属 | `/scout/odom`, `/scout/cmd_vel` |
| `/carrier/` | Carrier 专属 | `/carrier/battery`, `/carrier/supply_cmd` |
| `/specialist/` | Specialist 专属 | `/specialist/arm_cmd`, `/specialist/thermal_*` |
| `/swarm/` | 全局共享 | `/swarm/status`, `/swarm/markers` |
| `/gs/` | 地面站 | `/gs/command`, `/gs/command_feedback` |
| `/lora/` | LoRa 链路 | `/lora/cmd_vel`, `/lora/estop` |

### QoS 配置

控制指令和传感器数据使用不同 QoS：

```python
from qos_profiles import QOS_COMMAND, QOS_SENSOR

# 控制指令: 必须可靠送达, 保留最新 1 条
self.create_publisher(Twist, "/scout/cmd_vel", QOS_COMMAND)

# 传感器: 尽力送达, 丢弃旧数据不重传
self.create_publisher(Odometry, "/scout/odom", QOS_SENSOR)
```

### 运行测试

```bash
# 通信协议测试
cd ~/swarm_rescue/test
python test_protocol.py

# 运动学算法测试
python test_chassis_kinematics.py

# RL 环境测试
python test_disaster_env.py

# 全部测试
python -m pytest test/ -v
```

---

## 许可证

MIT License

