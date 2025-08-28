cd ~/inabot_ws/src
ros2 pkg create battery_can --build-type ament_python --dependencies rclpy std_msgs


inabot_ws/src/battery_can/
├─ package.xml
├─ setup.py
├─ battery_can/
│   ├─ __init__.py
│   └─ battery_can_node.py   ← Python 노드
└─ resource/

python3 -m pip install --user aenum
sudo apt install python3-can

colcon build --packages-select battery_can
source install/setup.bash
ros2 run battery_can battery_can_node

1) General Specifications

Protocol type: CAN 2.0A

Bitrate: 250 Kbps

Data length: 8 bytes per message

Message transmission period: 1 second

Connector: D-Sub 9-pin Female

Pin 2 → CAN L

Pin 7 → CAN H

Data format: Big Endian

sudo ip link set can0 up type can bitrate 250000
sudo ip link set can0 txqueuelen 1000  # optional, 큐 길이 늘리기
sudo ip link show can0

====================================

프로토콜: CAN 2.0A, 250kbps, 8bytes, Big Endian

메시지 주기: 1초

수신 처리:

0x101 → HW/FW, 용량, 잔여, 사이클

0x102 → 전압

0x103 → 전류, SOC, SOH, 셀 수

0x103 → 상태/보호/경고/밸런싱

0x104 → 온도

0x105~0x109 → 셀 전압들


1. sensor_msgs/msg/BatteryState 주요 필드

voltage : V (배터리 팩 전압)

current : A (방전은 음수, 충전은 양수 convention 권장)

charge : 현재 충전 용량 (Ah)

capacity : 전체 용량 (Ah)

percentage : SOC (0.0 ~ 1.0)

cell_voltage[] : 개별 셀 전압 (V 단위 권장, mV 아님)

temperature : °C

2. 현재 CAN 프로토콜 데이터 단위

팩 전압 : 0.1 V

팩 전류 : 0.1 A (signed)

팩 전체 용량 : 0.1 Ah

팩 잔여 용량 : 0.1 Ah

SOC : 0.1 %

SOH : 1 %

셀 전압 : mV

셀 온도 : 0.1 ℃

| Message ID   | 데이터 위치                             | 자료형 (Big Endian) | 단위 변환    | 설명                    |
| ------------ | ---------------------------------- | ---------------- | -------- | --------------------- |
| 0x101        | Byte\[0]                           | UInt8            | ×0.1     | Hardware version      |
| 0x101        | Byte\[1]                           | UInt8            | ×0.1     | Firmware version      |
| 0x101        | Byte\[3:2]                         | UInt16           | ÷10 → Ah | Pack 전체 용량            |
| 0x101        | Byte\[5:4]                         | UInt16           | ÷10 → Ah | Pack 잔여 용량            |
| 0x101        | Byte\[7:6]                         | UInt16           | 그대로      | Cycle count           |
| 0x102        | Byte\[1:0]                         | UInt16           | ÷10 → V  | Pack 전압               |
| 0x103        | Byte\[3:2]                         | Int16            | ÷10 → A  | 팩 전류 (충전 + / 방전 -)    |
| 0x103        | Byte\[5:4]                         | UInt16           | ÷10 → %  | SOC                   |
| 0x103        | Byte\[6]                           | UInt8            | %        | SOH                   |
| 0x103        | Byte\[7]                           | UInt8            | 그대로      | 직렬 셀 수                |
| 0x103        | Byte\[0]                           | UInt8            | 그대로      | Battery status        |
| 0x103        | Byte\[3:2]                         | UInt16           | 그대로      | Protection Flag       |
| 0x103        | Byte\[5:4]                         | UInt16           | 그대로      | Warning Flag          |
| 0x103        | Byte\[7:6]                         | UInt16           | 그대로      | Balancing Status      |
| 0x104        | Byte\[1:0]                         | Int16            | ÷10 → ℃  | 셀 온도 #1               |
| 0x104        | Byte\[3:2]                         | Int16            | ÷10 → ℃  | 셀 온도 #2               |
| 0x105\~0x109 | Byte\[1:0], \[3:2], \[5:4], \[7:6] | UInt16           | mV       | 셀 전압 1\~16 (메시지당 4개씩) |



| 데이터         | struct 코드 | 의미                          | 범위            |
| ----------- | --------- | --------------------------- | ------------- |
| 전류(Current) | `>h`      | Signed 16-bit, Big Endian   | -32768\~32767 |
| SOC, 전압, 용량 | `>H`      | Unsigned 16-bit, Big Endian | 0\~65535      |
| 셀 온도        | `>h`      | Signed 16-bit, Big Endian   | -32768\~32767 |
