Buzzard40 Bird Charger Node (CANopen)

1. 개요

[Master Node / ROS2] 
        │
        ▼
      [Bird]  <-- ROS2 Node에서 제어
        │
        ▼
      [Nest]  <-- 단순 전력 공급


Bird Node: ROS2에서 제어 필요
 -배터리 충전 상태, 전압/전류, Fault 확인
 -SDO 전송으로 충전 시작/정지

Nest Charger: ROS2에서 별도 제어 필요 없음
  -Bird가 접속하면 자동 전력 공급
  -필요 시 상태 모니터링 TPDO만 읽으면

        ┌───────────────┐
        │ Master Node   │  <-- ROS2 Node / Robot Controller
        │ (CANopen Master) │
        └───────┬───────┘
                │ CANopen
                ▼
        ┌───────────────┐
        │ Bird Module   │  <-- Robot Internal Charger Module
        │ (Node ID 0x64)│
        │ CANopen Matrix│
        └───────┬───────┘
                │ Battery / Status
                ▼
        ┌───────────────┐
        │ Nest Charger  │  <-- Station Charger
        │ (Node ID ?)   │
        │ CANopen Matrix│
        └───────────────┘


###

ROS2 버전: Humble 
목적: Buzzard40 Bird Charger 제어
프로토콜: CANopen

Node ID
Bird Node: 0x64
Master Node: 0x01
Baudrate: 기본 250Kbps (지원: 125K, 250K, 500K, 1M)
CAN 인터페이스: SocketCAN can0

Little Endian

EDS 파일: 필요 없음 (SDO/TPDO 주소를 코드에 직접 매핑)

2. 통신 구조
| 방향            | PDO   | COB-ID         | 설명              |
| ------------- | ----- | -------------- | --------------- |
| Master → Bird | RPDO1 | 0x180 + NodeID | System Command  |
| Bird → Master | TPDO1 | 0x182 + NodeID | System Status 1 |
| Bird → Master | TPDO2 | 0x282 + NodeID | System Status 2 |
| Heartbeat     | HB    | 1000ms         | Node 상태 알림      |
 
Heartbeat / NMT → 노드 상태 관리
RPDO / TPDO → 고속 주기 데이터 전송 (명령/상태)
SDO → Object Dictionary 접근, 설정/진단

3. 주요 기능
Heartbeat 모니터링: Node 상태 확인

TPDO/SDO 기반 상태 읽기:

충전 상태, 전류, 전압, 최대 전류, 출력 상태, Fault/Warning 번호 등

충전 제어:

SDO 명령으로 충전 시작/정지

오류 및 연결 대기:

CAN 버스 미연결 시 5초 간격 재시도 루프

1️⃣ NMT / Heartbeat

Master → Bird: NMT 상태 제어

Heartbeat:

0x00 Boot

0x7F Pre-operational

0x05 Operational

0x04 Stopped

구현:

Node Operational 전환 시

MASTER node
## RPDO1: Master → Bird (System Command 1)
Object	Index	Type  BE/LE	Info
Enable	0x4200-0	uint8	LE  0: Disconnect, 1: Connect
Reset	0x4201-0	uint8	LE  0: Reset faults off, 1: Reset faults on
Mode	0x4203-0	uint8	LE  0: Current control, 1: Voltage control
Charge Voltage Request	  0x2276-0	uint16	LE Unit = 1/256 V
Charge Current RequestE   0x6070-0	uint16	LE Unit = 1/16 A
Run	    0x4202-0	uint8	LE  0: Stop, 1: Start

buzzard-Bird node
## TPDO1: Bird → Master (System Status 1)
Object	Index	Type	BE/LE       Info
Actual Charging Current	0x2002-0	uint16	LE  Unit = 1/256 A
Actual Charging Voltage	0x2101-0	uint16	LE  Unit = 1/256 V
Max Available Charger Current	0x4212-0	LE  uint16	Unit = 1/16 A
Charger Output Status	0x2006-0	uint16	LE  Bit 15~12: 0=Off, 1=On

## TPDO2: Bird → Master (System Status 2)
Object	Index	Type	BE/LE       Info
Fault Number	0x2051-0	uint16	LE  Fault code table 참조
Warning Number	0x2050-0	uint8	LE  Warning code table 참조
Charger State	0x2007-0	uint8	LE  0: Waiting, 1: Ready, 2: Charging, 3: Fault
Alignment	0x2008-0	uint8	LE  0: Bad, 9: Very good
Thermal Usage	0x2009-0	uint8	LE  0: Good, 100: Derating
Actual Charging Power	0x2102-0	LE  uint16	Unit = 1/16 W


## MASTER - Heartbeat
Name      ObjectNr-Subindex Type      BE/LE     Bit info
NMT status   -              unit8     LE        0x7F: Pre-operational
                                                0x05: Operational
                                                0x04: Stopped
                                                0x00: Boot
RX - Heartbeat (Bird)
Name      ObjectNr-Subindex Type      BE/LE     Bit info
NMT status   -              unit8     LE        0x7F: Pre-operational
                                                0x05: Operational
                                                0x04: Stopped
                                                0x00: Boot

Non-PDO mapped objects
Object	                    Index	    Type	BE/LE       Info
Maximum charging voltage    0x4208-0    uint16  LE          Unit = (1/256)V
Application state           0x5003-1    uint8   LE          Reserved
                            0x5003-2    uint8   LE          Reserved
                            0x5003-3    uint8   LE          Reserved

Application state request   0x5004-1    uint8   LE          Reserved
                            0x5004-2    uint8   LE          Reserved
                            0x5004-3    uint8   LE          Reserved
DiagBlock                   0x5010-1    uint8 array [350]   LE      Reserved
                            0x5010-2    uint8 array [350]   LE      Reserved

DiagBlock Address           0x5011  uint8               Reserved

DiagBlock Ready           0x5011  uint8               Reserved
                                            (Used by Diagnostic tool)


## Bird 에러 메시지
1001 : Reverse current protection 
1002 : Battery disconnected during charging
1004 : Temperature sensor electronics 1
1005 : Temperature sensor electronics 2
1006 : Temperature sensor coil 1
1007 : Temperature sensor coil 2
1008 : Temperature sensor rectifier 1 
1009 : Temperature sensor rectifier 2
1012 : Overtemperature on electronics
1013 : Overtemperature on coil
1014 : Overtemperature on rectifier
1016 : No communication Bird <> Nest
1017 : Output contactor 1
1018 : Output contactor 2
1019 : Output contactor 3
1020 : Nest not found
1021 : Wireless link hardware failed
1022 : Pairing
1024 : Output voltage too low
1025 : Internal hardware
1031 : Can timeout
1033 : Overcurrent protection
1034 : AC leakage current
1041 : HW not enabled
1042 : Output voltage too high
1043 : Internal circuit clock failed
1053 : Internal DCDC voltage 1
1054 : Internal DCDC voltage 2
1057 : Internal current sensor reference
1058 : Internal fuse broken
1059 : Temperature sensor electronics 1
1060 : Temperature sensor electronics 2
1061 : Temperature sensor coil 1
1062 : Temperature sensor coil 2
1063 : Temperature sensor rectifier 1
1064 : Temperature sensor rectifier 2
1067 : Dump circuit not resettable
1068 : Reverse current circuit not resettable
1069 : Contactor circuit
1070 : Wireless link hardware failed
1075 : Overcurrent circuit not resettable
1076 : Reset by software protection watchdog

Bird 경고 메시지

001 : Derating by electronics temperature
002 : Derating by coil temperature
003 : Derating by rectifier temperature
 


1️⃣ Bird 모듈 Heartbeat 구조 

┌───────────────┐
│ Bird Module   │  Node ID 0x64
│ (Charger)     │
└───────┬───────┘
        │ CANopen TPDO / Heartbeat
        ▼
┌───────────────┐
│ ROS2 Node     │  Master 역할
│ (BirdCharger) │
└───────────────┘

CAN ID: 0x700 + NodeID → 0x764
데이터 길이: 1~8 bytes (Heartbeat는 1 byte만 사용)
데이터[0] → NMT 상태 (unit8, Little Endian)

| 값    | 상태              |
| ---- | --------------- |
| 0x00 | Boot            |
| 0x7F | Pre-operational |
| 0x05 | Operational     |
| 0x04 | Stopped         |


-------

데이터 검증 방법(더미 데이터-파이션)

1. Python 노드 실행:
/inabot_ws/python$ python3 dummy_charger_can_pub.py 

2. C++ 노드 실행:
    로그에 Heartbeat, TPDO1/2 값이 정상적으로 출력되는지 확인

    예시 출력:
        [inabot_core-2] [INFO] [1757400272.697417054] [buzzard40_bird_node]: [TPDO1] Current: 1.50 A, Voltage: 48.00 V, Max: 5.00 A, Output: 0x000F
        [inabot_core-2] [INFO] [1757400272.697683115] [buzzard40_bird_node]: [TPDO2] State: 2, Fault: 0, Warning: 0, Alignment: 1, Thermal: 20, Power: 72.00 W

    ros2 topic echo /can_tx

        can_id: 742
        data:
        - 2
        - 0
        - 0
        - 0
        - 1
        - 20
        - 128
        - 4
        can_dlc: 8
        ---
        can_id: 1892
        data:
        - 5
        can_dlc: 1
        ---
        can_id: 486
        data:
        - 128
        - 1
        - 0
        - 48
        - 80
        - 0
        - 15
        - 0
        can_dlc: 8

3. 테스트 시나리오
    | 단계 | 설명                                        |
    | -- | ----------------------------------------- |
    | 1  | Heartbeat Boot → Pre-op → Operational     |
    | 2  | Operational 상태 유지하면서 TPDO1/2 지속 전송        |
    | 3  | C++ 로그에서 값 확인                             |
    | 4  | 데이터 불일치, CAN ID, 길이 오류 발생 시 Python 메시지 수정 |
