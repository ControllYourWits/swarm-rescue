#!/usr/bin/env bash
set -euo pipefail

# ═══════════════════════════════════════════════════════════
# setup.sh — 一键环境安装脚本
# 用法: bash setup.sh [stm32|ros2|rl|all]
#
# 模式说明:
#   stm32 — 安装 ARM 工具链 + FreeRTOS 内核, 编译 STM32 固件
#   ros2  — 安装 ROS2 + 依赖, 构建工作空间
#   rl    — 安装 RL 训练依赖 (PyTorch, Stable-Baselines3)
#   all   — 以上全部
# ═══════════════════════════════════════════════════════════
MODE="${1:-all}"
GREEN="\033[0;32m"
NC="\033[0m"

info() {
  echo -e "${GREEN}[INFO]${NC} $1"
}

# ── STM32 固件工具链 ──────────────────────────────────────
install_stm32() {
  info "安装 STM32 ARM 工具链..."
  sudo apt update
  sudo apt install -y gcc-arm-none-eabi binutils-arm-none-eabi stlink-tools openocd make

  # 为每个机器人的固件下载 FreeRTOS 内核
  for robot in scout carrier specialist; do
    dir="firmware/${robot}"
    if [ ! -d "${dir}/Middlewares/FreeRTOS/Source" ]; then
      info "下载 FreeRTOS 内核: ${robot}"
      git clone --depth=1 https://github.com/FreeRTOS/FreeRTOS-Kernel.git \
        "${dir}/Middlewares/FreeRTOS/Source"
    fi
  done

  info "STM32 工具链就绪"
  echo "  编译 Scout:   cd firmware/scout && make -j\$(nproc)"
  echo "  烧录 Scout:   make flash"
}

# ── ROS2 工作空间 ─────────────────────────────────────────
install_ros2() {
  info "安装 ROS2 Humble 运行时依赖..."
  sudo apt update
  sudo apt install -y \
    ros-humble-desktop \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-xacro \
    ros-humble-tf2-ros \
    ros-humble-rplidar-ros \
    ros-humble-slam-toolbox \
    ros-humble-nav2-bringup \
    ros-humble-nav2-msgs \
    ros-humble-rosbridge-suite \
    ros-humble-cv-bridge \
    ros-humble-vision-msgs \
    ros-humble-v4l2-camera \
    python3-colcon-common-extensions \
    python3-pip

  pip3 install --user pyserial numpy onnxruntime opencv-python-headless requests flask flask-socketio
  sudo usermod -aG dialout "$USER"

  # 设置 ROS_DOMAIN_ID (三机通信域)
  grep -q "ROS_DOMAIN_ID" ~/.bashrc || echo "export ROS_DOMAIN_ID=42" >> ~/.bashrc

  info "构建 ROS2 工作空间 ~/ros2_ws"
  mkdir -p ~/ros2_ws/src
  cd ~/ros2_ws/src

  # 符号链接各 ROS2 包 (修改源码后无需重新拷贝)
  ln -sf "$(cd ../../; pwd)/scout"               scout
  ln -sf "$(cd ../../; pwd)/carrier"             carrier
  ln -sf "$(cd ../../; pwd)/specialist"          specialist
  ln -sf "$(cd ../../; pwd)/swarm_msgs"          swarm_msgs
  ln -sf "$(cd ../../; pwd)/swarm_bringup"       swarm_bringup
  ln -sf "$(cd ../../; pwd)/ground_station"      ground_station

  cd ~/ros2_ws
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  rosdep install --from-paths . --ignore-src -r -y
  colcon build --symlink-install

  info "ROS2 工作空间构建完成"
  echo "  激活环境: source ~/ros2_ws/install/setup.bash"
  echo "  仿真模式: ros2 launch swarm_bringup sim.launch.py"
  echo "  实车模式: ros2 launch swarm_bringup swarm.launch.py"
}

# ── RL 强化学习环境 ───────────────────────────────────────
install_rl() {
  info "安装 RL 训练依赖..."
  pip3 install --user -r rl_training/requirements.txt
  info "RL 环境就绪"
  echo "  快速训练: cd rl_training && python3 train_scout.py --fast"
  echo "  评估模型: python3 eval_scout.py --model checkpoints/best/best_model.zip"
}

# ── 主入口 ────────────────────────────────────────────────
case "$MODE" in
  stm32) install_stm32 ;;
  ros2)  install_ros2 ;;
  rl)    install_rl ;;
  all)   install_stm32; install_ros2; install_rl ;;
  *)     echo "用法: bash setup.sh [stm32|ros2|rl|all]"; exit 2 ;;
esac

info "完成. 部分操作需要重新登录 (dialout 权限)."
