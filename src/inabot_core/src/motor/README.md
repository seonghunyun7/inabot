

[PC (Ubuntu 20.04/22.04)]
└── scoketCAN (can0 @ 500kbps)
     ├── Motor Driver 1 (Node ID: 0x01, 0x02)
     │    └── Wheel Motor 1 (with encoder)
     │    └── Wheel Motor 2 (with encoder)
     └── Motor Driver 2 (Node ID: 0x03, 0x04)
          └── Wheel Motor 3 (with encoder)
          └── Wheel Motor 4 (with encoder)

    통신 프로토콜: CAN (Controller Area Network)

    통신 속도: 500 kbps

    각 드라이버는 2개의 휠 모터 제어

    PC의 can0 포트에 총 4개의 노드 ID (0x01~0x04) 가 붙어 있고,

    이들은 같은 CAN 버스 상에 존재하면서 각자 다른 ID를 사용해 통신한다는 의미입니다.

    즉, 하나의 socket (can0) 으로 이 모든 ID들과 송수신이 가능합니다.

🔌 CAN 프로토콜 특성상:

    하나의 CAN 인터페이스 (can0) 에 여러 장치 (Node ID) 를 연결하는 것은 정상적인 사용 방식입니다.

    각 장치는 서로 다른 ID를 가지며, PC는 수신 시 해당 ID를 보고 메시지를 분기 처리할 수 있습니다.

✔ 예를 들어

당신이 만든 CanInterface 하나가 열려 있다면:

    수신된 프레임은 아래처럼 ID에 따라 구분 가능합니다:

struct can_frame frame;
read(socket_fd_, &frame, sizeof(struct can_frame));

if (frame.can_id == 0x01) {
    // Motor 1 관련 처리
} else if (frame.can_id == 0x03) {
    // Motor 3 관련 처리
}

또는 각 MotorDriver 인스턴스가 자신이 담당할 ID 범위를 갖고 필터링만 잘 해주면 됩니다.
🔧 Tip: CAN 필터 적용도 가능

CAN 필터(setsockopt + CAN_RAW_FILTER)를 쓰면, 특정 ID만 수신하게 제한할 수도 있습니다. 예:

struct can_filter rfilter[2];
rfilter[0].can_id = 0x01;
rfilter[0].can_mask = CAN_SFF_MASK;
rfilter[1].can_id = 0x02;
rfilter[1].can_mask = CAN_SFF_MASK;

setsockopt(socket_fd_, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));

이렇게 하면 0x01, 0x02만 수신합니다.
다중 모터를 나누어 관리하려면 유용합니다.


✅ 2. 개발 단계 요약
📌 STEP 1. CAN 통신 환경 구축

    can-utils, socketcan 등을 활용한 CAN 인터페이스 구성

    candump로 기본 통신 가능 여부 확인

📌 STEP 2. CAN 데이터 수집 및 테이블화

    candump can0 > log.txt 명령어로 데이터 수집

    Python 스크립트를 활용해 로그를 CSV로 파싱 및 정리

    테이블화: Timestamp, CAN ID, Data, 각 Byte 의미 추론

📌 STEP 3. CAN 메시지 의미 해석

    CAN ID 별 메시지 목적 (명령 vs 상태 피드백) 식별

    데이터 구조 및 Byte 단위 해석

        속도, 위치, 전류, 오류 등

    단위 및 스케일링 정보 확보

📌 STEP 4. C++ 기반 송수신 코드 구현

    can_frame 구조체로 프레임 구성 및 송신 (write)

    프레임 수신 후 CAN ID에 따라 해석 (read)

    정기 송신 및 수신을 위한 스레드 / 타이머 구성

📌 STEP 5. ROS2 연동 (선택 사항)

    ROS2 노드에서 cmd_vel, odom 등을 처리

    CAN 제어 모듈을 ROS2 패키지로 구성

    실시간 피드백 시각화: rqt_plot, rviz, rosbag

📌 STEP 6. 정량적 테스트 및 검증

    엔코더 값의 정확도 및 일관성 평가

    명령 속도 → 실제 응답 속도 매칭 여부 확인

    Odometry 계산 정확도

    통신 이상 시 안전 정지 여부

✅ 3. CAN Dump 데이터 분석 및 테이블화 절차
절차	설명
1. 데이터 수집	candump로 통신 로그 확보
2. 파싱 및 CSV 정리	Timestamp, CAN ID, Data 등 정리
3. Byte 분해	Data 필드를 1바이트 단위로 나눔
4. 의미 추론	실제 동작과 비교해 각 Byte 의미 해석
5. 구조화 테이블 작성	CAN ID 별로 메시지 이름, Byte 역할, 단위 등 명시
예시 테이블
CAN ID	메시지 종류	Byte 구조	의미
0x200 + NodeID	속도 명령	Byte0-1: 속도값 (int16), Byte2: 모드	목표 속도 전송
0x210 + NodeID	속도 피드백	Byte0-1: 현재 속도, Byte2-3: 전류	상태 모니터링
✅ 4. C++ 구현 흐름 예시

struct can_frame frame;
frame.can_id = 0x200 + node_id; // 속도 명령
frame.can_dlc = 8;
frame.data[0] = (speed >> 8) & 0xFF; // MSB
frame.data[1] = speed & 0xFF;        // LSB
// ... 기타 데이터 세팅
write(can_socket, &frame, sizeof(frame));

수신 처리:

read(can_socket, &frame, sizeof(frame));
if ((frame.can_id & 0xFF0) == 0x210) {
    // 상태 피드백 수신 처리
    int16_t actual_speed = (frame.data[0] << 8) | frame.data[1];
}

⚠️ 5. 개발 시 어려운 점 및 주의 사항
항목	설명	주의 사항
① 메시지 구조 해석의 어려움	CAN ID 및 데이터 Byte의 의미를 추론해야 함	매뉴얼/DBC 파일 없으면 테스트 결과와 함께 해석 필요
② 엔디안 및 부호 처리	Big/Little endian, signed 여부	값이 왜곡될 수 있으니 반드시 확인 필요
③ 단위 및 스케일링 파악	실제 단위 변환이 불명확	물리 실험 (거리 측정 등)으로 검증 필요
④ 로그 해석의 반복성	추론-테스트-재해석의 반복	영상/로그/조건 정리를 꼼꼼히
⑤ CAN 통신 안정성 문제	메시지 손실, 오동작 위험	수신 타임아웃, 재전송 로직, 정지 제어 포함 필요
⑥ 멀티 모터 동기성 확보	4개 모터가 동시에 동일하게 응답해야 함	응답 시간 편차 최소화 구조 필요
⑦ 제어 루프 안정성	ROS 루프가 블로킹되면 전체 제어 불안정	멀티스레드 구조, select/poll 사용 고려
✅ 6. 정량적 테스트 체크리스트
항목	기대 결과	테스트 방법
속도 응답 시간	< 200ms	cmd_vel → 속도 피드백 수렴 시간 측정
직선 주행 거리	±3% 이내 오차	실제 거리 ↔ odom 비교
회전 정확도	±3도 이내	자이로 or RViz 비교
슬립 여부	없음	휠 회전 ↔ 실제 거리 비교
CAN 장애 감지	100ms 이내	can0 차단 후 알람 발생
cmd_vel 중단 시 감속	자동 정지	1초간 명령 중단 후 동작 확인
멀티 모터 동기화	< 10ms 편차	각 모터 응답 시간 비교


✅ 2. Roboteq 모터 드라이버 CAN 제어의 기본 구성

Roboteq는 CAN을 통한 명령 및 상태 피드백을 지원합니다. 일반적으로 다음 구조로 구성됩니다:
✅ 명령 예시 (TX)

    속도 명령 (0x200 + NodeID)

    조향 각도 또는 위치 명령 (0x201 + NodeID)

    운전 모드 설정, 상태 요청 등

✅ 피드백 예시 (RX)

    상태 피드백 (0x210 + NodeID)

    실제 속도, 전류, 에러 상태 등

✅ 3. 구현 흐름 (C++)
1. CAN 프레임 구성

can_frame frame;
frame.can_id = 0x200 + node_id;  // 예: 속도 명령
frame.can_dlc = 8;
frame.data[0] = ...;  // 속도값 MSB
frame.data[1] = ...;  // 속도값 LSB
...
write(can_socket, &frame, sizeof(frame));

2. 수신 처리

read(can_socket, &frame, sizeof(frame));
if ((frame.can_id & 0xFF0) == 0x210) {
    // 상태 응답 처리
}

3. 테이블을 참고한 매핑

    테이블에서 정리된 CAN ID별 데이터 구조를 기반으로
    각 프레임을 정확히 해석하고 구성합니다.

    예:

int16_t speed = (frame.data[0] << 8) | frame.data[1]


Setup CAN Controller
====================
.. _quick-start-setup-can-controller:

**Option 1**: Virtual CANController

.. code-block:: console

  $ sudo modprobe vcan
  $ sudo ip link add dev vcan0 type vcan
  $ sudo ip link set vcan0 txqueuelen 1000
  $ sudo ip link set up vcan0

**Option 2**: Peak CANController

.. code-block:: console

  $ sudo modprobe peak_usb
  $ sudo ip link set can0 up type can bitrate 1000000
  $ sudo ip link set can0 txqueuelen 1000
  $ sudo ip link set up can0

Bitrate depends on your bus and devices capabilities.

**Option 3**: candleLight USB-CAN Adapter

.. code-block:: console

  $ sudo modprobe gs_usb
  $ sudo ip link set can0 up type can bitrate 500000
  $ sudo ip link set can0 txqueuelen 1000
  $ sudo ip link set up can0

Bitrate depends on your bus and devices capabilities.

**Option 4**: Adapt these steps to other socketcan devices



# CAN 인터페이스 재설정
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up

🔐 루트 권한 없이 자동 실행하고 싶다면?
➤ /etc/sudoers 에서 NOPASSWD 설정이 필요합니다 (주의 요망)

ysh@ysh-ThinkPad-T16-Gen-4:~$ whoami
ysh
ysh@ysh-ThinkPad-T16-Gen-4:~$ 


sudo visudo

ysh ALL=(ALL) NOPASSWD: /sbin/ip

위 설정을 통해 ip 명령에 대해 비밀번호 입력 없이 sudo 실행이 가능합니다. 하지만 보안상 매우 주의해서 사용해야 하며, 테스트 환경이나 제한된 시스템에서만 적용할 것을 권장합니다.

===========================================
typedef __u32 canid_t;
struct can_frame {
    canid_t can_id;    // 32비트 CAN ID + 플래그 (RTR, EFF, ERR 포함)
    union {
        __u8 len;      // 유효한 데이터 길이 (0~8바이트) ← 현재 사용되는 필드
        __u8 can_dlc;  // 이전 이름 (deprecated)
    } __attribute__((packed)); // 구조체 정렬 무시 (패딩 방지)
    
    __u8 __pad;        // 정렬용 패딩 (일부 ABI에서 필요)
    __u8 __res0;       // 예약됨 (현재 사용 안 함)
    __u8 len8_dlc;     // CAN FD 지원 시 사용 (9~15바이트)
    
    __u8 data[CAN_MAX_DLEN] __attribute__((aligned(8))); 
                       // 실제 데이터 (최대 8바이트, 정렬됨)
};


┌────────────────────┐
│   MotorDriver      │     (최상위 제어 클래스)
└────────┬───────────┘
         │
         ▼
┌─────────────────────────────┐
│  std::shared_ptr<ICanInterface> can_driver_ 
└────────┬────────────────────┘
         │
         │ (다형성 기반 호출)
         ▼
 ┌────────────────────────────┐
 │      SocketCanDriver       │
 │ ┌───────────────────────┐ │
 │ │  ICanInterface        │◄┘  (Interface)
 │ └───────────────────────┘
 └────────────────────────────┘
 
          또는
 
 ┌────────────────────────────┐
 │      PeakCanDriver         │
 │ ┌───────────────────────┐  │
 │ │  ICanInterface        │ ◄┘  (Interface)
 │ └───────────────────────┘  |
 └────────────────────────────┘


MotorDriver::init() 안에서
         │
         ▼
MotorControllerFactory::create(type, can_driver_);

         │
         ▼
 ┌────────────────────────────────────────────────────┐
 │  std::shared_ptr<IMotorController> motor_controller│
 └───────────────┬────────────────────────────────────┘
                 │
                 ▼
      ┌──────────────────────────────┐
      │   RoboteqMotorController     │  ◄─── "roboteq"
      │   DummyMotorController       │  ◄─── "dummy" <-dumy test 용
      │   xxxxxMotorController        │  ◄─── "xxxxxx"
      └──────────────────────────────┘
             ▲             ▲             ▲
             │             │             │
             └─────────────┴─────────────┘
             (공통 부모)
             IMotorController


[MotorDriver]
  |
  |----> has-a ----> std::shared_ptr<ICanInterface>  → SocketCanDriver 또는 PeakCanDriver
  |
  |----> uses ----> MotorControllerFactory::create(type, can_driver_)
                      |
                      └──> returns shared_ptr<IMotorController>
                                |
                                ├─> RoboteqMotorController
                                ├─> DummyMotorController
                                └─> xxxxxMotorController


✅ 3. 계층 구조 요약
계층	클래스	설명
🔝 애플리케이션	MotorDriver	전체 모터 제어 흐름 담당
🔁 팩토리 계층	MotorControllerFactory	모터 컨트롤러 동적 생성
🧠 모터 컨트롤러	RoboteqMotorController, DummyMotorController	실제 제어 로직 담당
📡 통신 드라이버	SocketCanDriver	CAN 통신 처리
🔌 인터페이스	ICanInterface, IMotorController	추상화 계층
✅ 4. 디자인 패턴 요약
패턴	사용 위치	설명
Factory	MotorControllerFactory	컨트롤러 인스턴스를 동적 생성
Interface 기반 DI	ICanInterface, IMotorController	유연한 구조 및 테스트 용이성
RAII + 스마트 포인터	모든 컴포넌트	메모리/리소스 안전성 확보

+---------------------+
|    struct can_frame  |
+---------------------+
| can_id   : 32 bits   |  <-- CAN ID + 플래그 (표준/확장/오류 등)
+---------------------+
| can_dlc  : 8 bits    |  <-- 데이터 길이 (0~8)
+---------------------+
| __pad    : 8 bits    |  <-- 패딩 (사용자에 따라 무시 가능)
+---------------------+
| __res0   : 8 bits    |  <-- 예약 필드 (사용 안함)
+---------------------+
| __res1   : 8 bits    |  <-- 예약 필드 (사용 안함)
+---------------------+
| data[8] : 8*8 bits   |  <-- 실제 CAN 데이터 (최대 8바이트)
+---------------------+


각 필드 설명
필드명	크기	설명
can_id	32 bits	11/29비트 CAN ID 및 플래그
can_dlc	8 bits	데이터 길이 (0~8)
__pad	8 bits	패딩 (사용자에 따라 무시 가능)
__res0	8 bits	예약 필드
__res1	8 bits	예약 필드
data	8 bytes	실제 CAN 메시지 데이터


데이터 예시

data = [0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x64]

이 배열에서

    data[0] = 0x00

    data[1] = 0x00

    data[2] = 0x10

    data[3] = 0x00

    data[4] = 0x00

    data[5] = 0x00

    data[6] = 0x00

    data[7] = 0x64

parseInt32FromData(&data[0]) 계산

함수 내부를 다시 보자면:

return static_cast<int32_t>(
    (static_cast<uint32_t>(d[0]) << 24) |  // 가장 상위 바이트 (MSB)
    (static_cast<uint32_t>(d[1]) << 16) |
    (static_cast<uint32_t>(d[2]) << 8)  |
    static_cast<uint32_t>(d[3])           // 가장 하위 바이트 (LSB)
);

d는 data 배열의 시작 주소인데, 첫 번째 호출은 &data[0] 입니다.

따라서 값을 계산해보면:

(d[0] << 24) = 0x00 << 24 = 0x00000000
(d[1] << 16) = 0x00 << 16 = 0x00000000
(d[2] << 8)  = 0x10 << 8  = 0x00001000
d[3]        = 0x00       = 0x00000000
------------------------------
합계 = 0x00001000 (16진수) = 4096 (10진수)

parseInt32FromData(&data[4]) 계산

이번에는 d가 &data[4] 즉, data[4]부터 시작합니다:

(d[0] << 24) = 0x00 << 24 = 0x00000000
(d[1] << 16) = 0x00 << 16 = 0x00000000
(d[2] << 8)  = 0x00 << 8  = 0x00000000
d[3]        = 0x64       = 0x00000064
------------------------------
합계 = 0x00000064 (16진수) = 100 (10진수)

요약

    배열의 첫 4바이트 (data[0]~data[3])를 빅 엔디안 방식으로 해석해 32비트 정수로 변환했기 때문에
    0x00001000 = 4096 이 됩니다.

    배열의 다음 4바이트 (data[4]~data[7])도 같은 방식으로 해석해
    0x00000064 = 100 이 됩니다.

참고: 빅 엔디안 해석 방식

    가장 높은 바이트가 첫 번째 바이트에 위치 (MSB first)

    가장 낮은 바이트가 마지막 바이트에 위치 (LSB last)