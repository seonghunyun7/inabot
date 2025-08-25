inabot_joy

inabot_joy 조이스틱 또는 게임패드를 사용하여 lrbot2 로봇을 수동으로 제어할 수 있도록 해주는 ROS2 패키지입니다. 이 패키지는 joy 노드로부터 입력을 받아 이를 로봇의 제어 명령으로 변환하는 기능을 수행합니다.
📦 주요 구성 요소

    소스 코드 (src/)

        joy_control.cpp: 조이스틱 입력을 받아 Twist 메시지로 변환하여 퍼블리시하는 노드

    런치 파일 (launch/)

        joy_start.launch.py: 조이스틱 제어를 시작하기 위한 런치 파일

    CMakeLists.txt: 패키지 빌드를 위한 CMake 설정 파일

    package.xml: 패키지 메타데이터 및 의존성 관리 파일

📂 디렉토리 구조

├── CMakeLists.txt
├── launch
│   └── joy_start.launch.py
├── package.xml
├── README.md
└── src
    └── joy_control.cpp

🧑‍💻 주요 기능 설명
1. joy_control.cpp

    ROS2의 sensor_msgs/msg/Joy 메시지를 구독하여 조이스틱 입력을 수신합니다.

    입력된 축(axis)과 버튼(button) 값을 기반으로 로봇의 이동 명령(geometry_msgs/msg/Twist)을 생성하고 퍼블리시합니다.

    일반적으로 왼쪽 스틱은 직진/후진 및 좌우 회전을 담당합니다.

2. joy_start.launch.py

    조이스틱 장치 노드(joy_node)와 joy_control 노드를 함께 실행할 수 있도록 설정하는 런치 파일입니다.

    조이스틱 종류에 따라 /dev/input/jsX 또는 매핑 설정이 필요할 수 있습니다.

🚀 패키지 빌드

이 패키지를 빌드하려면, ROS2 워크스페이스의 루트 디렉토리에서 다음 명령어를 실행하세요:

colcon build --packages-select inabot_joy
colcon build --symlink-install --packages-select inabot_joy

빌드 후 환경을 설정합니다:

source install/setup.bash

🕹️ 사용 예시
1. 런치 파일을 통한 실행

ros2 launch inabot_joy joy_start.launch.py

2. 조이스틱 매핑 설정

기본적으로 Xbox 컨트롤러 또는 PS4 패드 기준으로 작성되어 있으며, 사용 환경에 따라 joy_control.cpp 내 매핑 설정을 수정해야 할 수 있습니다.
🔧 의존성

    sensor_msgs: 조이스틱 입력 메시지

    geometry_msgs: 로봇의 이동 명령 메시지

    rclcpp, std_msgs: ROS2 기본 메시지 및 노드 실행

📌 참고

    조이스틱이 인식되지 않으면 ls /dev/input/ 또는 jstest 명령어로 연결 상태를 확인하세요.

    udev 규칙이나 권한 설정이 필요할 수 있습니다 (/dev/input/js0 접근 권한 등).