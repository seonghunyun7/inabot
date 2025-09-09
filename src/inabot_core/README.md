inabot_core

inabot_core는 ROS 2 기반 모터 제어 및 주행 알고리즘 패키지로, 로봇의 운동학, 모터 제어, 센서 처리, 오도메트리, 경로 계획 등을 포함합니다.

📁 프로젝트 구조
inabot_core/
│ │ ├── differential_drive.hpp
│ │ ├── mecanum_drive.hpp
│ │ ├── omni_drive.hpp
│ │ ├── quaternion_drive.hpp
│ │ ├── steerable_wheel_drive.hpp
│ │ └── kinematics_factory.hpp
│ ├── motor/ # 모터 제어 추상화 및 CAN 인터페이스
│ │ ├── can_driver/
│ │ ├── controller/
│ │ ├── kinematics/
│ │ ├── motor.hpp
│ │ └── utils/
│ ├── odom/ # 오도메트리
│ │ └── odometry.hpp
│ ├── planner/ # 경로 계획 관련 공용 헤더
│ │ ├── cam_obstacle/
│ │ ├── common/
│ │ ├── global/
│ │ └── local/
│ ├── sensor/ # 센서 인터페이스 (카메라, LiDAR)
│ │ ├── cam/ # QR, ArUco
│ │ └── lidar/ # LiDAR
│ └── utils/ # 공용 유틸리티
│ ├── crash_handler.hpp
│ └── logger.h
│
├── src/ # 소스 구현
│ ├── core.cpp # 노드 초기화 및 실행
│ ├── imu/
│ │ ├── imu.cpp
│ │ └── SerialInterface.cpp
│ ├── kinematics/
│ │ ├── differential_drive.cpp
│ │ ├── mecanum_drive.cpp
│ │ ├── omni_drive.cpp
│ │ ├── quaternion_drive.cpp
│ │ └── steerable_wheel_drive.cpp
│ ├── motor/
│ │ ├── can_driver/
│ │ ├── controller/
│ │ ├── kinematics/
│ │ ├── motor.cpp
│ │ └── utils/
│ ├── odom/
│ │ └── odometry.cpp
│ ├── planner/
│ │ ├── cam_obstacle/
│ │ ├── common/
│ │ ├── global/
│ │ └── local/
│ ├── sensor/
│ │ ├── cam/
│ │ └── lidar/
│ └── utils/
│ ├── crash_handler.cpp
│ └── logger.cpp
│
├── config/
│ └── params.yaml # 파라미터 설정 파일
├── launch/
│ └── inabot_core.launch.py # ROS 2 런치 파일
├── CMakeLists.txt
├── package.xml
├── core.drawio # 아키텍처 다이어그램
├── core.png # 아키텍처 이미지
└── README.md


📁 주요 컴포넌트
1. CAN 인터페이스

include/motor/can_driver/

SocketCAN 기반 기본 API 제공

모터 컨트롤러별 명령 프로토콜 구현 필요

2. 운동학 (Kinematics)

include/kinematics/

다양한 주행 알고리즘 포함

로봇 특성에 따라 수정/확장 가능

3. 모터 제어

src/motor/ + include/motor/

CAN 명령 송신, PID 제어, 엔코더 피드백 처리

4. 센서 처리

IMU, LiDAR, 카메라 기반 마커 인식

오도메트리와 결합하여 위치 추정 지원

5. 경로 계획

글로벌 플래너: 전체 경로 생성 (A*, Dijkstra 등)

로컬 플래너: 실시간 경로 추종 및 속도/조향 제어 (PID, MPC)

📁 빌드 및 실행 방법
# 워크스페이스 최상위에서
colcon build --packages-select inabot_msgs
colcon build --packages-select inabot_description 
source ~/.bashrc

colcon build --packages-select inabot_core
source install/setup.bash

# 런치 실행
ros2 launch inabot_core inabot_core.launch.py

📁 클래스 간 데이터 흐름
cmd_vel callback
   │
   ▼
kinematics::BaseKinematics::twistToWheelVelocities()
   │
   ▼
motor_kinematics::toPulseVelocity()  (기어비, CPR 변환)
   │
   ▼
motor_driver::processPidMotion()     (PID 제어 + CAN 송신)
   │
   ▼
모터 구동
   │
   ▼
모터 엔코더 신호 → motor_driver::parseEncoderFrame()
   │
   ▼
현재 속도 피드백 → PID 입력

kinematics: 순수 수학적 운동학 변환

motor_kinematics: 하드웨어 특성 변환 (기어비, CPR)

motor_driver: 실제 제어, PID, CAN 통신 담당

📁 주의 사항

CAN 명령은 모터 제조사 규격에 맞게 직접 구현 필요

운동학 모듈은 로봇별 최적화 필요

파라미터 설정은 config/params.yaml 참조
