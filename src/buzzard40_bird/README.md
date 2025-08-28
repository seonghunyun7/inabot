Buzzard40 Bird Charger Node (CANopen) – 참조용 문서

1. 개요

ROS2 버전: Humble

언어: Python

목적: Buzzard40 Bird Charger 제어

프로토콜: CANopen

Node ID

Bird Node: 0x64

Master Node: 0x01

Baudrate: 기본 250Kbps (지원: 125K, 250K, 500K, 1M)

CAN 인터페이스: SocketCAN can0

EDS 파일: 필요 없음 (SDO/TPDO 주소를 코드에 직접 매핑)

2. 통신 구조
| 방향            | PDO   | COB-ID         | 설명              |
| ------------- | ----- | -------------- | --------------- |
| Master → Bird | RPDO1 | 0x180 + NodeID | System Command  |
| Bird → Master | TPDO1 | 0x182 + NodeID | System Status 1 |
| Bird → Master | TPDO2 | 0x282 + NodeID | System Status 2 |
| Heartbeat     | HB    | 1000ms         | Node 상태 알림      |


3. 필수 패키지 설치
python3 -m pip install --user python-can canopen aenum
python-can: CAN 하드웨어 인터페이스
canopen: CANopen 프로토콜 구현
aenum: 상태 코드 enum 정의


4. 빌드 및 실행
# ROS2 워크스페이스 이동
cd ~/inabot_ws

# buzzard40_bird 패키지만 빌드
colcon build --packages-select buzzard40_bird

# ROS2 환경 설정
source install/setup.bash

# Node 실행
ros2 run buzzard40_bird bird_node

5. 주요 기능
Heartbeat 모니터링: Node 상태 확인

TPDO/SDO 기반 상태 읽기:

충전 상태, 전류, 전압, 최대 전류, 출력 상태, Fault/Warning 번호 등

충전 제어:

SDO 명령으로 충전 시작/정지

오류 및 연결 대기:

CAN 버스 미연결 시 5초 간격 재시도 루프