# Inabot

inabot는 ROS 2 Humble 환경에서 사용되는 **메타 패키지(Meta Package)**입니다.
이 패키지는 Inabot 로봇 시스템의 다양한 기능을 포함하는 서브 패키지들을 통합 관리합니다.

---

## 📦 포함된 패키지

| 패키지 이름             | 설명 |
| 패키지 이름                 | 설명                            |
| ---------------------- | ----------------------------- |
| `inabot_core`          | 로봇 제어 핵심 노드                   |
| `inabot_description`   | URDF 및 로봇 모델 정의               |
| `inabot_joy`           | 조이스틱 입력 처리 노드                 |
| `inabot_teleop`        | 키보드/조이스틱 텔레옵 노드               |
| `inabot_msgs`          | 사용자 정의 메시지                    |
| `inabot_slam`          | SLAM(자율지도작성) 관련 설정 및 노드       |
| `inabot_acs`           | 자율 주행/제어 관련 노드                |
| `robot_upstart`        | 시스템 서비스 실행 지원 (부팅 시 자동 실행)    |
| `orbbec_camera`\*      | Orbbec 카메라 드라이버 (*선택적*)       |
| `orbbec_camera_msgs`\* | Orbbec 카메라 메시지 정의 (*선택적*)     |
| `orbbec_description`\* | Orbbec 카메라 URDF/모델 정의 (*선택적*) |

---

## 🔧 빌드 방법

# ROS 2 Humble 환경 설정
source /opt/ros/humble/setup.bash

# 워크스페이스 이동 및 빌드
cd ~/inabot_ws
colcon build --symlink-install --packages-select inabot

# 빌드 후 환경 설정
source install/setup.bash

ℹ️ 참고

inabot 메타 패키지는 실행 가능한 노드가 없으며, 포함된 서브 패키지들을 함께 관리하기 위한 용도입니다.

센서/드라이버 패키지(orbbec_camera, LiDAR 드라이버 등)는 로봇 하드웨어 구성에 따라 선택적으로 추가할 수 있습니다.