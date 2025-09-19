#!/bin/bash
# Orbbec ROS2 Wrapper 환경 설정 (Ubuntu 22.04 + ROS 2 Humble)
# 실행 방법: chmod +x setup_orbbec_ros2.sh && ./setup_orbbec_ros2.sh

set -e  # 오류 발생 시 스크립트 종료

# 1. ROS 2 Humble 의존성 패키지 설치
echo "[1/2] Installing required packages..."
sudo apt update
sudo apt install -y libgflags-dev nlohmann-json3-dev \
ros-humble-image-transport ros-humble-image-transport-plugins ros-humble-compressed-image-transport \
ros-humble-image-publisher ros-humble-camera-info-manager \
ros-humble-diagnostic-updater ros-humble-diagnostic-msgs ros-humble-statistics-msgs \
ros-humble-backward-ros libdw-dev

# 2. udev rules 적용
echo "[2/2] Applying udev rules for Orbbec cameras..."
cd ~/inabot_ws/src/OrbbecSDK_ROS2/orbbec_camera/scripts
sudo bash install_udev_rules.sh
sudo udevadm control --reload-rules
sudo udevadm trigger

echo "All done! You can now build the ROS2 workspace with colcon."

