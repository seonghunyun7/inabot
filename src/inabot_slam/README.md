inabot_slam

inabot_slam ROS2 기반의 SLAM(Simultaneous Localization and Mapping) 패키지로, Google Cartographer를 이용해 2D LiDAR 및 Odometry, IMU 데이터를 활용한 실시간 지도 작성 및 위치 추정을 수행합니다.
📦 주요 구성 요소

    config/
    다양한 센서 조합을 위한 Cartographer 설정 파일 모음 (.lua)

    launch/
    실제 맵핑 수행 및 시뮬레이션을 위한 launch 파일

    rviz/
    RViz에서 SLAM 결과를 시각화하기 위한 설정 파일

    CMakeLists.txt & package.xml
    빌드 및 패키지 정의 파일

📂 디렉토리 구조

inabot_slam/
├── CMakeLists.txt
├── config/
│   ├── cartographer_2d_lidar.lua
│   ├── cartographer_2d_lidar_test.lua
│   ├── cartographer_2d_odom.lua
│   ├── cartographer_2d_odom_imu.lua
│   └── ...
├── launch/
│   ├── carto_mapping.launch.py
│   ├── carto_sim_mapping.launch.py
│   └── __init__.py
├── package.xml
├── README.md
└── rviz/
    └── slam.rviz

🚀 사용법
1. 빌드

colcon build --packages-select inabot_slam
colcon build --symlink-install --packages-select inabot_slam
source install/setup.bash

2. 실시간 SLAM 실행 (예: 2D LIDAR + Odom)

ros2 launch inabot_slam carto_mapping.launch.py

ros2 launch inabot_slam carto_sim_mapping.launch.py bag_filename:=/home/ysh/bags/test_bag.db3

    bag_filename:=<경로> 형식으로 지정

    예제에서는 /home/ysh/hs_bag/hs_0313_bag_0.db3

3. 시뮬레이션용 SLAM 실행

ros2 launch inabot_slam carto_sim_mapping.launch.py

⚙️ 설정 파일 설명 (config/)

    cartographer_2d_lidar.lua
    LIDAR만 사용하는 기본적인 설정

    cartographer_2d_odom.lua
    LIDAR + Odometry를 활용하는 설정

    cartographer_2d_odom_imu.lua
    LIDAR + Odometry + IMU를 조합한 설정

    demo_config.lua
    테스트용 데모 환경 설정

    일부 파일은 테스트/복사본(copy.lua)으로 추정됩니다. 정식 설정 파일을 기준으로 사용하세요.

🧭 RViz 사용

rviz2 -d $(ros2 pkg prefix inabot_slam)/share/inabot_slam/rviz/slam.rviz

🔧 의존성

    cartographer_ros

    robot_state_publisher

    tf2_ros, rviz2

    기타 센서 관련 패키지 (sensor_msgs, nav_msgs, geometry_msgs 등)

📌 참고사항

    IMU 또는 Odometry 유무에 따라 .lua 파일 선택이 중요합니다.

    launch 파일에서 사용 중인 .lua는 수정 없이도 기본 동작이 가능하나, 센서 위치/노이즈 수준에 따라 튜닝이 필요할 수 있습니다.

    실기기에서는 센서 데이터 topic이 올바르게 설정되어 있어야 정상적으로 SLAM이 작동합니다.


sudo apt update
sudo apt install ros-humble-cartographer
sudo apt install ros-humble-cartographer-ros 
 