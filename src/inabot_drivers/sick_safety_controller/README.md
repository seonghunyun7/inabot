Flexi Soft Safety Controller는 모듈식 구조로 다양한 필드버스 및 산업 네트워크를 지원합니다. 
IPC(Industrial PC)와 인터페이스를 구성할 때 주로 사용하는 방식은 통신 프로토콜과 연결 방식  

1️⃣ IPC와의 주요 인터페이스
| 구분                          | Flexi Soft 지원 | 특징 / 용도                          |
| --------------------------- | ------------- | -------------------------------- |
| **Ethernet/IP™**            | 지원            | 산업용 이더넷, PLC 및 IPC와 안전 데이터 통신 가능 |
| **PROFINET / PROFIBUS DP**  | 지원            | Siemens PLC 및 IPC 연결 시 많이 사용     |
| **EtherCAT®**               | 지원            | 빠른 실시간 제어, 주로 Motion Control에 사용 |
| **Modbus TCP/RTU**          | 지원            | 범용 산업용 통신, 간단한 데이터 전송용           |
| **CANopen**                 | 지원            | 모터, 드라이브, 센서 등 제어용               |
| **DeviceNet™ / CC-Link**    | 지원            | 일부 레거시 PLC/드라이브 연결용              |
| **EFI / EFI-pro**           | 지원            | SICK 안전 센서와 안전 신호 전용 통신          |
| **Flexi Link / Flexi Line** | 지원            | Flexi Soft 모듈 간 안전 데이터 캐스케이드 연결  |


2️⃣ 일반적인 IPC 인터페이스 선택 기준
안전 신호 필요 → EFI-pro: SICK 안전 센서 데이터 직접 인터페이스
PLC와 일반 데이터 통신 → Ethernet/IP, PROFINET, Modbus
실시간 제어/모션 제어 → EtherCAT
모듈 간 확장 연결 → Flexi Link / Flexi Line

 
IPC에서 직접 안전 데이터를 처리해야 한다면 Ethernet/IP Safety 혹은 EFI-pro 사용
안전 관련 제어 로직은 Safety Designer 소프트웨어에서 구성 후 통신 설정
각 모듈별로 IP 주소, Node ID, Slave ID 등 설정 필요


🔹 IPC ↔ Flexi Soft 연결 예시
 ┌─────────────┐          ┌───────────────────────┐
 │             │          │                       │
 │    IPC      │ <──────> │ Flexi Soft Safety     │
 │ (Industrial │ Ethernet │ Controller (Modular) │
 │    PC)      │          │                       │
 └─────────────┘          └─────────┬─────────────┘
                                      │
                       ┌──────────────┴───────────────┐
                       │                              │
                  ┌───────────┐                  ┌───────────┐
                  │ Main      │                  │ Gateway   │
                  │ Module    │                  │ Module    │
                  └───────────┘                  └───────────┘
                       │                              │
        ┌──────────────┴───────────────┐
        │                              │
  ┌─────────────┐                ┌─────────────┐
  │ Expansion   │                │ Relay       │
  │ Module(s)   │                │ Module(s)   │
  └─────────────┘                └─────────────┘
        │                              │
  ┌─────────────┐                ┌─────────────┐
  │ Flexi Loop  │                │ EFI / EFI-pro│
  │ (Safety     │                │ (SICK Safety │
  │ Sensors)    │                │ Devices)     │
  └─────────────┘                └─────────────┘

🔹 연결 설명

IPC ↔ Flexi Soft Main Module
Ethernet/IP, PROFINET, Modbus TCP 등 산업용 이더넷으로 통신
안전 관련 데이터는 “Safety over Ethernet” 프로토콜 사용 가능

Main Module ↔ Expansion / Relay Module
모듈 간 플러그/슬롯 연결 (Flexi Link / Flexi Line)
안전 센서 및 릴레이 제어 신호 캐스케이드

Flexi Loop
안전 센서를 직렬/캐스케이드 연결하여 신호 통합
글로벌 비상정지(E-Stop) 및 안전 입력 처리

EFI / EFI-pro
SICK 안전 센서 통신용 전용 프로토콜
PLC나 IPC에서 안전 신호 읽을 때 사용



cd ~/inabot_ws
colcon build --packages-select sick_safety_controller
source install/setup.bash
ros2 launch sick_safety_controller safety_controller.launch.py


==

1️⃣ libmodbus 설치
sudo apt update
sudo apt install libmodbus-dev


3️⃣ 정리된 추정 IO 구성
구분	Modbus 레지스터	비트 위치	기능 추정
Safety Input	256 (regs[0])	0..15	일반 안전 입력 (예: 센서, E-stop)
EMO / 상태	257 (regs[1])	0..15	긴급 정지, 오류 플래그 등
Safety Output	258 (regs[2])	0..15	릴레이 출력, 충전 제어 등
출력 예시	258	7	충전 관련 제어 (TYPE_CHARGE)


┌─────────────────────────────┐
│  Safety Controller (PLC)    │
│      Flexi Soft              │
├─────────────┬───────────────┤
│ 레지스터    │ 설명          │
├─────────────┼───────────────┤
│ 256 (regs[0]) │ Safety Inputs │
│               │ IN[0..15]    │
│               │ - 비상정지    │
│               │ - 도어 센서   │
│               │ - 안전 라이트 │
│               │ ...           │
├─────────────┼───────────────┤
│ 257 (regs[1]) │ EMO / 상태    │
│               │ EMO[0..15]   │
│               │ - 긴급 정지   │
│               │ - 오류 플래그 │
│               │ - 상태 모니터 │
├─────────────┼───────────────┤
│ 258 (regs[2]) │ Safety Outputs│
│               │ OUT[0..15]   │
│               │ - 릴레이 출력│
│               │ - 충전 ON/OFF │
│               │ - 기타 출력  │
└─────────────┴───────────────┘



┌───────────────────────────────────────────────┐
│           SICK Flexi Soft PLC                │
│             (Modbus TCP 256~258)             │
├───────────────┬───────────────┬──────────────┤
│ Register 256  │ Register 257  │ Register 258 │
│ Safety Inputs │ EMO / 상태    │ Safety Output│
├───────────────┼───────────────┼──────────────┤
│ IN[0]         │ EMO[0]        │ OUT[0]       │
│ - E-Stop      │ - EMO Active  │ - Relay 0    │
│ IN[1]         │ EMO[1]        │ OUT[1]       │
│ - Door sensor │ - Error Flag  │ - Relay 1    │
│ IN[2]         │ EMO[2]        │ OUT[2]       │
│ - Light curtain│ ...          │ - Relay 2    │
│ IN[3]         │ ...           │ ...          │
│ ...           │ ...           │ ...          │
│ IN[15]        │ EMO[15]       │ OUT[15]      │
│ - Safety sensor │ - Misc Status│ - Charge     │
└───────────────┴───────────────┴──────────────┘

| C++ 변수 / 함수                       | Modbus 레지스터 | 비트    | 역할              |
| --------------------------------- | ----------- | ----- | --------------- |
| `readInputs()` → states\[0]       | 256         | 0..15 | Safety 입력 읽기    |
| `readInputs()` → states\[1]       | 257         | 0..15 | EMO / 상태 읽기     |
| `readInputs()` → states\[2]       | 258         | 0..15 | Safety 출력 상태 읽기 |
| `writeOutput(TYPE_CHARGE, value)` | 258         | 7     | 충전 출력 제어        |


┌───────────────┐        TCP/IP         ┌───────────────┐
│  IPC (ROS 2)  │ <-----------------> │ Flexi Soft PLC│
│  SafetyController Node │             │ Modbus Server │
│  Parameters: IP, Port │             │ Registers 256~258 │
└───────────────┘                       └───────────────┘


[ Flexi Soft Main Module ]
┌─────────────────────────────────────────────┐
│        Main CPU Module (Modbus TCP/IP)      │
│  - IP: 192.168.0.10                         │
│  - Port: 502                                │
├───────────────┬───────────────┬─────────────┤
│ Reg 256       │ Reg 257       │ Reg 258     │
│ Safety Inputs │ EMO / Status  │ Safety Out  │
├───────────────┼───────────────┼─────────────┤
│ IN[0]         │ EMO[0]        │ OUT[0]      │
│ - E-Stop      │ - EMO Active  │ - Relay 0   │
│ IN[1]         │ EMO[1]        │ OUT[1]      │
│ - Door sensor │ - Error Flag  │ - Relay 1   │
│ IN[2]         │ EMO[2]        │ OUT[2]      │
│ - Light Curtain│ ...           │ - Relay 2   │
│ IN[3]         │ ...           │ ...         │
│ IN[4]         │ ...           │ ...         │
│ IN[5]         │ ...           │ ...         │
│ IN[6]         │ ...           │ ...         │
│ IN[7]         │ EMO[7]        │ OUT[7]      │
│ - Safety Sensor│ ...           │ - Charge    │
│ IN[8..15]     │ EMO[8..15]    │ OUT[8..15]  │
└───────────────┴───────────────┴─────────────┘


[ Flexi Soft I/O Expansion Modules ]
┌─────────────────────────────────────────────┐
│  Safety Input Module(s)                     │
│  - DI[0..15]                                 │
│  - 연결: Main Module via backplane          │
├─────────────────────────────────────────────┤
│  Safety Output Module(s)                    │
│  - DO[0..15]                                 │
│  - 릴레이 출력 / 경고 신호                  │
└─────────────────────────────────────────────┘


┌───────────────┐        TCP/IP         ┌──────────────────────┐
│  IPC (ROS 2)  │ <-----------------> │ Flexi Soft Main CPU  │
│ SafetyController Node │              │ Modbus Server         │
│ - IP: 192.168.0.10   │              │ Registers 256~258    │
│ - Port: 502          │              │ Inputs/Outputs       │
└───────────────┘                      └──────────────────────┘

🔹 매핑 요약 (Module / Register / Bit)
| Module                 | Register | Bit   | Function                       |
| ---------------------- | -------- | ----- | ------------------------------ |
| Main Module            | 256      | 0..15 | Safety Inputs (E-Stop, Sensor) |
|                        | 257      | 0..15 | EMO / Status (Error Flags)     |
|                        | 258      | 0..15 | Safety Outputs (Relay, Charge) |
| Safety Output Module 1 | 259\~260 | 0..15 | Additional DO                  |
| Safety Input Module 1  | 261\~262 | 0..15 | Additional DI                  |
