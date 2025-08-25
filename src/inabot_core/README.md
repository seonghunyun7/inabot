📁 프로젝트 개요 및 구조

inabot_core는 ROS 2 기반 모터 제어 및 주행 알고리즘 패키지입니다.

inabot_core/
├── include/         # 헤더 파일 (CAN 인터페이스, 운동학, 모터 등)
├── src/             # 소스 코드 (모터, 운동학, IMU, 오도메트리 등)
├── config/          # 파라미터 설정 파일
├── launch/          # 런처 파일
├── CMakeLists.txt
└── package.xml

📁 주요 컴포넌트

    CAN 인터페이스 (can_interface.hpp)
    SocketCAN 기반 기본 API만 제공하며, 모터 컨트롤러별 명령 프로토콜에 맞춘 구현이 필요합니다.

    운동학 (kinematics/)
    기본 주행 운동학 알고리즘 포함, 로봇 유형과 특성에 따라 알고리즘 수정 또는 확장 필요.

    모터 제어 (motor/)
    CAN을 통한 명령 전송 및 인코더 데이터 수신 담당.

    IMU, 오도메트리, 기타 유틸리티
    센서 데이터 처리와 주행 위치 계산 지원.


📁 빌드 및 실행 방법

# 워크스페이스 최상위 디렉토리에서
colcon build --packages-select inabot_core
source install/setup.bash

# 노드 직접 실행
ros2 run inabot_core inabot_core_node

# 또는 런처 실행
ros2 launch inabot_core inabot_core.launch.py


📁 주의 사항

    CAN 명령은 모터 제조사 규격에 맞게 직접 구현해야 하며, 기본 API는 단순 전송 기능만 제공합니다.

    운동학 모듈은 로봇별로 최적화된 알고리즘 적용이 필요합니다.

    파라미터 설정은 config/params.yaml 파일을 참고하세요.


control_toolbox
sudo apt update
sudo apt install ros-humble-control-toolbox


PID 제어기와 내비게이션 스택 2의 역할 구분
1. 글로벌 플래너(Global Planner)

    경로 생성: 시작점에서 목표점까지의 전체 경로를 생성

    주로 맵 기반이고, 장애물 회피 및 최적 경로 탐색 알고리즘 사용 (예: A*, Dijkstra)

    경로는 일반적으로 궤적 포인트들의 시퀀스

2. 로컬 플래너(Local Planner)

    즉각적 움직임 제어: 글로벌 플래너가 준 경로를 따라 실제 로봇이 움직일 수 있도록 명령 생성

    주행 중 동적 장애물 회피, 속도/방향 미세 조정 포함

    PID, MPC, DWA 같은 제어기/알고리즘 사용 가능

PID 제어기의 위치

    PID는 로컬 플래너에서 속도 및 조향 각도 제어에 쓰입니다.

    로봇이 목표 경로에 따라 실제로 주행할 수 있게 모터 명령을 생성하는 역할.

정리
역할	위치	예시 알고리즘
글로벌 플래너	상위 레벨 경로 생성	A*, Dijkstra, Navfn, 글로벌 RRT
로컬 플래너	실시간 경로 추종 및 제어	PID, DWA, TEB, MPC
결론

    PID 제어기 = 로컬 플래너 내부에서 하위 제어기로 작동하는 컴포넌트

    PID 자체가 전체 경로를 생성하지는 않음




5. 클래스 간 관계 간단 요약

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

요약

    kinematics: 순수 수학적 운동학 변환

    motor_kinematics: 하드웨어 특성(기어비, CPR 등) 변환

    motor_driver: 실제 제어, PID, CAN 통신 담당




    inabot_core/
├── include/                   # 공용 헤더 파일들
│   ├── motor.hpp             # 모터 제어 추상화
│   ├── odometry.hpp          # Odometry 클래스 인터페이스
│   ├── motor_encoder_data.hpp        # WheelData, WheelChannel 정의
│   ├── motor_kinematics/     # 모터 운동학 관련 헤더
│   │   ├── motor_kinematics.hpp
│   │   └── motor_utils.hpp
│   ├── motion/               # 제어기, 속도 프로파일
│   │   ├── pid_controller.hpp
│   │   └── velocity_profile.hpp
│   ├── monitor/              # 시스템 상태 모니터링
│   │   ├── monitor.hpp
│   │   ├── network_monitor.hpp
│   │   └── system_monitor.hpp
│   ├── imu.hpp               # IMU 인터페이스
│   ├── can_interface.hpp     # CAN 통신 추상화
│   ├── kinematics/           # 다양한 주행 방식 인터페이스
│   │   ├── base_kinematics.hpp
│   │   ├── differential_drive.hpp
│   │   ├── mecanum_drive.hpp
│   │   ├── omni_drive.hpp
│   │   ├── quaternion_drive.hpp
│   │   ├── steerable_wheel_drive.hpp
│   │   └── kinematics_factory.hpp       # 주행 모델 선택용 팩토리
│   └── utils/                # 공용 유틸리티
│       ├── crash_handler.hpp
│       └── logger.h

├── src/                      # 소스 구현
│   ├── core.cpp              # 노드 초기화 및 실행
│   ├── odom/                 # Odometry 관련 구현
│   │   └── odometry.cpp
│   ├── imu/                  # IMU 처리
│   │   └── imu.cpp
│   ├── motor/                # 모터 드라이버 + CAN 인터페이스
│   │   ├── can_interface.cpp
│   │   └── motor.cpp
│   ├── motor_kinematics/     # 모터 운동학 변환
│   │   ├── motor_kinematics.cpp
│   │   └── motor_utils.cpp
│   ├── motion/               # 제어기 구현
│   │   ├── pid_controller.cpp
│   │   └── velocity_profile.cpp
│   ├── monitor/              # 모니터링
│   │   ├── monitor.cpp
│   │   ├── network_monitor.cpp
│   │   └── system_monitor.cpp
│   ├── kinematics/           # 주행 모델 구현
│   │   ├── differential_drive.cpp
│   │   ├── mecanum_drive.cpp
│   │   ├── omni_drive.cpp
│   │   ├── quaternion_drive.cpp
│   │   ├── steerable_wheel_drive.cpp
│   │   └── factory.cpp
│   ├── filters/              # (비어 있음: 필터 관련 예정)
│   ├── sensors/              # (비어 있음: 센서 관련 예정)
│   └── utils/                # 유틸리티
│       ├── crash_handler.cpp
│       ├── logger.cpp

├── config/
│   └── params.yaml           # 파라미터 설정 파일

├── launch/
│   └── inabot_core.launch.py  # ROS 2 런치 파일

├── CMakeLists.txt
├── package.xml
├── core.drawio              # 아키텍처 다이어그램 (설계용)
├── core.png                 # 아키텍처 이미지
├── README.md
└── README copy.md           # 백업된 README
