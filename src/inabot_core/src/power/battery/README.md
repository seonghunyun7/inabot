# BMS CAN Node (ROS2 Humble)

## 개요

이 노드는 BMS(Battery Management System)와 CAN 통신을 통해 배터리 상태를 수집하고 ROS2 토픽으로 publish합니다.

- **Protocol**: CAN 2.0A, 250Kbps, 8 bytes
- **Message 주기**: 1초
- **Connector**: DSub-9pin Female (Pin2 – CAN L / Pin7 – CAN H)
- **Data Format**: Big Endian
- **CAN 인터페이스가 없을 경우** 안전하게 재시도하며 계속 대기
- **Publish Topic**: `battery_state` (`inabot_msgs/msg/BmsState`)

---

## CAN 메시지 구조

### 0x101: Pack basic info

| Byte | 자료형    | 의미          | 변환       |
| ---- | -------- | ----------- | -------- |
| 0    | UInt8    | HW Version  | ×0.1     |
| 1    | UInt8    | FW Version  | ×0.1     |
| 2-3  | UInt16   | Pack 전체 용량  | ÷10 → Ah |
| 4-5  | UInt16   | Pack 잔여 용량  | ÷10 → Ah |
| 6-7  | UInt16   | Cycle count | 그대로   |

### 0x102: Pack voltage

| Byte | 자료형    | 의미      | 변환      |
| ---- | -------- | ------- | ------- |
| 0-1  | UInt16   | Pack 전압 | ÷10 → V |

### 0x103: Current / SOC / SOH / Serial / Flags

| Byte | 자료형    | 의미               | 변환      |
| ---- | -------- | ---------------- | ------- |
| 0    | UInt8    | Battery status   | 그대로  |
| 2-3  | Int16    | Current          | ÷10 → A |
| 4-5  | UInt16   | SOC              | ÷100 → % |
| 6    | UInt8    | SOH              | %       |
| 7    | UInt8    | Serial cells     | 그대로  |
| 2-3  | UInt16   | Protection Flag  | 그대로  |
| 4-5  | UInt16   | Warning Flag     | 그대로  |
| 6-7  | UInt16   | Balancing Status | 그대로  |

> ⚠️ 주의: Current, SOC, Flags가 겹치므로 실제 파싱 시 의미에 맞게 읽어야 합니다.

### 0x104: Cell temperature

| Byte | 자료형   | 의미      | 변환      |
| ---- | -------- | ------- | ------- |
| 0-1  | Int16    | Cell #1 | ÷10 → ℃ |
| 2-3  | Int16    | Cell #2 | ÷10 → ℃ |

### 0x105~0x109: Cell voltages  

각 메시지에 4개 셀 전압 포함  

- Byte 순서: `[1:0], [3:2], [5:4], [7:6]`  
- UInt16, 단위 mV → float V로 변환 시 `/1000.0f`  
- 0x105 프레임에서 vector 초기화, 이후 0x106~0x109는 이어서 push  

---

## ROS2 메시지 구조

`inabot_msgs/msg/BmsState`:

| Field                | Type      | 설명                        |
|----------------------|-----------|----------------------------|
| battery_id           | uint8     | 배터리 ID                  |
| hardware_ver         | float     | 하드웨어 버전              |
| firmware_ver         | float     | 펌웨어 버전                |
| capacity             | float     | 배터리 용량 (Ah)           |
| charge               | float     | 남은 용량 (Ah)             |
| cycle_count          | uint16    | 충방전 사이클 수            |
| voltage              | float     | 전압 (V)                   |
| current              | float     | 전류 (A)                   |
| percentage           | float     | SOC (%)                     |
| soh                  | uint8     | SOH                         |
| serial_cells         | uint8     | 직렬 셀 수                  |
| temperature          | float     | 평균 셀 온도 (°C)           |
| cell_voltage         | float[]   | 각 셀 전압 (V)              |
| battery_status       | uint8     | 배터리 상태                 |
| protection_flag      | uint16    | 보호 플래그                 |
| warning_flag         | uint16    | 경고 플래그                 |
| balancing_status     | uint16    | 밸런싱 상태                 |

---

## 데이터 변환 예시

```cpp
// 0x101 Pack basic info
hardware_ver = data[0]*0.1f;
firmware_ver = data[1]*0.1f;
capacity = toUInt16BE(&data[2])/10.0f;
charge = toUInt16BE(&data[4])/10.0f;
cycle_count = toUInt16BE(&data[6]);

// 0x102 Pack voltage
voltage = toUInt16BE(&data[0])/10.0f;

// 0x103 Current / SOC / SOH / Serial / Flags
current = toInt16BE(&data[2])/10.0f;
percentage = toUInt16BE(&data[4])/100.0f;
soh = data[6];
serial_cells = data[7];

// 0x104 Cell temperature
float t1 = toInt16BE(&data[0])/10.0f;
float t2 = toInt16BE(&data[2])/10.0f;
temperature = (t1+t2)/2.0f;

// 0x105~0x109 Cell voltages
if (base_id == 0x105) cell_voltage.clear();
for (size_t i = 0; i < 8; i += 2)
    cell_voltage.push_back(toUInt16BE(&data[i])/1000.0f);

⚠️ 주의: 0x105에서 cell_voltage.clear() 필요, 0x106~0x109는 이어서 push
         Big Endian 변환 유의
         UInt16/Int16 변환 시 toUInt16BE() / toInt16BE() 사용


## 토픽 확인
ros2 topic echo /battery_state

---
battery_id: 1
hardware_ver: 1.5
firmware_ver: 2.799999952316284
cycle_count: 13
soh: 115
serial_cells: 110
battery_status: 154
protection_flag: 12120
warning_flag: 34188
balancing_status: 29550
---
voltage: 45.599998474121094
current: 1212.0
capacity: 48.79999923706055
charge: 30.600000381469727
percentage: 341.8800048828125
temperature: 22.799999237060547
cell_voltage:
- 3.5959999561309814
- 3.5239999294281006
- 3.6110000610351562
- 3.694999933242798
- 3.8529999256134033
- 3.691999912261963
- 3.502000093460083
- 3.384999990463257
- 3.2760000228881836
- 3.7060000896453857
- 3.806999921798706
- 3.5250000953674316
- 3.7260000705718994
- 3.3369998931884766
- 3.632999897003174
- 4.129000186920166
- 4.197999954223633
- 3.5439999103546143
- 3.749000072479248
- 3.8919999599456787
battery_id: 1
hardware_ver: 1.5
firmware_ver: 2.799999952316284
cycle_count: 13
soh: 115
serial_cells: 110
battery_status: 154
protection_flag: 12120
warning_flag: 34188
balancing_status: 29550
---
