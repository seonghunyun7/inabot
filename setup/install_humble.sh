#!/bin/bash

# 1. 시스템 업데이트 및 필수 패키지 설치
sudo apt update && sudo apt upgrade -y
sudo apt install -y curl gnupg2 lsb-release locales build-essential

# 2. Locale 설정
locale  # check for UTF-8
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings

# 3. ROS 2 저장소 추가
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 4. ROS 2 Humble 설치
sudo apt update
sudo apt install -y ros-humble-desktop \
  ros-humble-rmw-fastrtps* \
  ros-humble-rmw-cyclonedds*

# 5. ROS 환경 설정
#echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
#source ~/.bashrc

# 6. 개발 툴 및 Python 패키지 설치
sudo apt update && sudo apt install -y \
  build-essential \
  cmake \
  git \
  libbullet-dev \
  python3-colcon-common-extensions \
  python3-flake8 \
  python3-pip \
  python3-pytest-cov \
  python3-rosdep \
  python3-setuptools \
  python3-vcstool \
  wget

python3 -m pip install -U \
  argcomplete \
  flake8-blind-except \
  flake8-builtins \
  flake8-class-newline \
  flake8-comprehensions \
  flake8-deprecated \
  flake8-docstrings \
  flake8-import-order \
  flake8-quotes \
  pytest-repeat \
  pytest-rerunfailures \
  pytest

# 7. 기타 의존성 설치
sudo apt install --no-install-recommends -y \
  libasio-dev \
  libtinyxml2-dev \
  libcunit1-dev

sudo apt update

# 워크스페이스 생성
mkdir -p ~/inabot_ws/src
cd ~/inabot_ws

# (원하는 패키지를 src에 복사하거나 clone 후)
sudo rosdep init
rosdep update
rosdep install --from-paths . --ignore-src --rosdistro humble -y

# 9. 추가 ROS 패키지 설치
sudo apt install ros-humble-cartographer-ros
sudo apt install ros-humble-nav2-bringup
sudo apt install ros-humble-joy
sudo apt install ros-humble-visualization-msgs
sudo apt install ros-humble-laser-filters
sudo apt install ros-humble-laser-geometry
sudo apt install ros-humble-message-filters
sudo apt install ros-humble-robot-localization
sudo apt install ros-humble-xacro
sudo apt install ros-humble-joint-state-publisher
sudo apt install ros-humble-gazebo-ros
sudo apt install ros-humble-gazebo-ros-pkgs
sudo apt install ros-humble-pcl-ros
sudo apt install ros-humble-vision-msgs
sudo apt install ros-humble-imu-filter-madgwick
sudo apt install ros-humble-visualization-msgs
sudo apt install libopencv-dev python3-opencv
sudo apt install ros-humble-control-toolbox

#sudo apt install ros-dev-tools -y
sudo apt install libboost-all-dev
sudo apt install libboost-stacktrace-dev

# 12. (선택) 편의 유틸 설치
sudo apt install -y terminator  htop tree net-tools

# 완료 메시지
echo ""
echo "====================================================="
echo " ROS 2 Humble 개발 환경 설정 완료!"
echo " 터미널을 새로 열거나 'source ~/.bashrc' 실행"
echo "====================================================="
