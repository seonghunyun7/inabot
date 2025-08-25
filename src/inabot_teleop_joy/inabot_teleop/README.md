inabot_teleop

inabot_teleop 키보드 또는 다른 입력 장치를 통해 로봇을 수동으로 제어할 수 있도록 하는 ROS2 기반 텔레옵(Teleoperation) 패키지입니다.
📁 디렉토리 구성

inabot_teleop/
├── CHANGELOG.rst              # 변경 이력
├── inabot_teleop/             # 메인 파이썬 코드 모듈
├── package.xml                # ROS2 패키지 설정
├── README.md                  # 설명 문서
├── resource/                  # 패키지 리소스 파일 (icon 등)
├── setup.cfg                  # 빌드 설정 파일
├── setup.py                   # Python entry point 정의

🚀 주요 기능

    ROS2 Topic을 통해 로봇의 속도 제어 (geometry_msgs/msg/Twist)

    키보드 또는 GUI를 통해 조작 가능 (Python 기반)

    ROS2에서 teleop_twist_keyboard 스타일과 유사한 텔레옵 인터페이스 제공 가능

📦 설치 및 빌드

cd ~/robot_ws
colcon build --packages-select inabot_teleop
colcon build --symlink-install --packages-select inabot_teleop
source install/setup.bash

🧪 실행 방법
예시: 키보드 텔레옵 실행 (해당 스크립트가 있다면)

ros2 run inabot_teleop teleop_keyboard

또는 Launch 파일을 통한 실행

(해당 패키지에 launch 파일이 추가되었다면)

ros2 run inabot_teleop teleop_keyboard

📚 의존 패키지

    rclpy

    geometry_msgs

    (선택적으로 keyboard, inputs, pygame 등 사용자 입력 관련 모듈 필요 시)

✏️ 개발 노트

inabot_teleop/ 디렉토리 내부에 주요 로직이 들어 있으며, setup.py를 통해 실행 가능한 entry point가 정의되어 있을 수 있습니다. 텔레옵 기능이 잘 작동하려면 로봇의 velocity topic(cmd_vel 등)과 정확히 일치해야 합니다.
