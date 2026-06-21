#!/usr/bin/env bash
set -euo pipefail

# Usage: bash setup.sh [stm32|ros2|rl|all]
MODE="${1:-all}"
GREEN="\033[0;32m"
NC="\033[0m"

info() {
  echo -e "${GREEN}[INFO]${NC} $1"
}

install_stm32() {
  info "Installing STM32 toolchain..."
  sudo apt update
  sudo apt install -y gcc-arm-none-eabi binutils-arm-none-eabi stlink-tools openocd make

  for robot in scout carrier specialist; do
    dir="${robot}/stm32_fw"
    if [ ! -d "${dir}/Middlewares/FreeRTOS/Source" ]; then
      info "Downloading FreeRTOS kernel for ${robot}"
      git clone --depth=1 https://github.com/FreeRTOS/FreeRTOS-Kernel.git \
        "${dir}/Middlewares/FreeRTOS/Source"
    fi
  done

  info "STM32 toolchain ready"
  echo "  Build Scout: cd scout/stm32_fw && make -j\$(nproc)"
  echo "  Flash Scout: make flash"
}

install_ros2() {
  info "Installing ROS 2 Humble runtime dependencies..."
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

  pip3 install --user pyserial numpy onnxruntime opencv-python-headless requests
  sudo usermod -aG dialout "$USER"

  grep -q "ROS_DOMAIN_ID" ~/.bashrc || echo "export ROS_DOMAIN_ID=42" >> ~/.bashrc

  info "Building ROS 2 workspace at ~/ros2_ws"
  mkdir -p ~/ros2_ws/src
  rm -rf ~/ros2_ws/src/scout ~/ros2_ws/src/carrier ~/ros2_ws/src/specialist \
         ~/ros2_ws/src/swarm_msgs ~/ros2_ws/src/swarm_bringup ~/ros2_ws/src/ground_station
  cp -r scout/ros2_pkg ~/ros2_ws/src/scout
  cp -r carrier/ros2_pkg ~/ros2_ws/src/carrier
  cp -r specialist/ros2_pkg ~/ros2_ws/src/specialist
  cp -r swarm_msgs ~/ros2_ws/src/swarm_msgs
  cp -r swarm_bringup ~/ros2_ws/src/swarm_bringup
  cp -r ground_station ~/ros2_ws/src/ground_station

  cd ~/ros2_ws
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  colcon build --symlink-install

  info "ROS 2 workspace built"
  echo "  Activate: source ~/ros2_ws/install/setup.bash"
  echo "  Simulation: ros2 launch swarm_bringup sim.launch.py"
}

install_rl() {
  info "Installing RL training dependencies..."
  pip3 install --user -r rl_training/requirements.txt
  info "RL environment ready"
  echo "  Train: cd rl_training && python3 train_scout.py --fast"
}

case "$MODE" in
  stm32) install_stm32 ;;
  ros2) install_ros2 ;;
  rl) install_rl ;;
  all) install_stm32; install_ros2; install_rl ;;
  *) echo "Usage: bash setup.sh [stm32|ros2|rl|all]"; exit 2 ;;
esac

info "Done. Re-login may be required for dialout permissions."
