#include "dummy_can_driver.hpp"

//#include <utils/logger.h>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <cstring>
#include <linux/can.h>
#include <cmath>

namespace Inabot {

DummyCanDriver::DummyCanDriver()
: is_open_(false) {}

DummyCanDriver::~DummyCanDriver() {
    close();
}

bool DummyCanDriver::initialize(const std::string& interface_name) {
    std::lock_guard<std::mutex> lock(mutex_);
    is_open_ = true;
    #if _LOG_
    std::cout << "[DummyCanDriver] Initialized on interface: " << interface_name << std::endl;
    #endif
    return true;
}

void DummyCanDriver::close() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (is_open_) {
        std::cout << "[DummyCanDriver] Closed interface" << std::endl;
        is_open_ = false;
    }
}

bool DummyCanDriver::sendFrame(uint32_t can_id, const std::vector<uint8_t>& data) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!is_open_) {
        std::cout << "[DummyCanDriver] Send failed: interface not open" << std::endl;
        return false;
    }

    std::cout << "[DummyCanDriver] sendFrame called with CAN ID: 0x"
              << std::hex << can_id << std::dec
              << " Data Size: " << data.size() << std::endl;
    return true;
}

/*
| 바퀴 구분    | Node ID | CAN ID               | 데이터 내용                       |
| -------- | ------- | -------------------- | ---------------------------- |
| 전륜 구동 바퀴 | 0x01    | 0x100 + 0x01 → 0x101 | traction position + velocity |
| 전륜 조향 바퀴 | 0x02    | 0x100 + 0x02 → 0x102 | steering position + velocity |
| 후륜 구동 바퀴 | 0x03    | 0x100 + 0x03 → 0x103 | traction position + velocity |
| 후륜 조향 바퀴 | 0x04    | 0x100 + 0x04 → 0x104 | steering position + velocity |

프레임 생성 방식

    CAN ID: 0x100 | node_id
    예) node_id=0x01 → CAN ID=0x101, node_id=0x02 → CAN ID=0x102, ...

    DLC: 8 (데이터 8바이트)

    데이터:

바이트	내용
0~3	위치 값 (int32_t)
4~7	속도 값 (int32_t)

    위치 값: 1000 * (frame_index + 1)
    → 1번째 프레임 pos=1000, 2번째 pos=2000, 3번째 pos=3000, 4번째 pos=4000

    속도 값: 100 * (frame_index + 1)
    → 1번째 vel=100, 2번째 vel=200, 3번째 vel=300, 4번째 vel=400

parseInt32FromData()를 이용해 데이터를 다시 32비트 정수로 복원합니다.

    첫 번째 프레임 (node_id=0x01, CAN ID=0x101)

        위치: 1000

        속도: 100

    두 번째 프레임 (node_id=0x02, CAN ID=0x102)

        위치: 2000

        속도: 200

    세 번째 프레임 (node_id=0x03, CAN ID=0x103)

        위치: 3000

        속도: 300

    네 번째 프레임 (node_id=0x04, CAN ID=0x104)

        위치: 4000

        속도: 400
*/
#if 0 //pulses per second 
::can_frame DummyCanDriver::readFrame() {
    std::lock_guard<std::mutex> lock(mutex_);
    ::can_frame frame{};

    if (!is_open_) {
        std::cout << "[DummyCanDriver] readFrame failed: interface not open" << std::endl;
        return frame;
    }

    static int frame_index = 0;
    const uint8_t node_ids[4] = {0x01, 0x02, 0x03, 0x04};
    static int32_t positions[4] = {0, 0, 0, 0};

    const int control_hz = 100;        // readFrame 호출 주기 (100Hz = 10ms)
    const int num_nodes = 4;
    const int node_update_hz = control_hz / num_nodes; // 각 노드별 업데이트 주기

    const double wheel_radius = 0.05;      // m
    const int pulse_per_revolution = 10000;  // 펄스 수

    // 0.3 m/s 속도 가정
    const double desired_speed_mps = 0.3;
    double circumference = 2.0 * M_PI * wheel_radius;  // 원둘레

    // 초당 펄스 수 계산 (speed / 한 펄스 거리)
    int ticks_per_sec = static_cast<int>(desired_speed_mps / (circumference / pulse_per_revolution));

    // 1프레임당 시간 (초) = 1 / node_update_hz
    double frame_time_sec = 1.0 / static_cast<double>(node_update_hz);

    // 누적 위치 업데이트 (펄스 단위)
    positions[frame_index] += static_cast<int32_t>(ticks_per_sec * frame_time_sec);
    int32_t pos = positions[frame_index];

    // velocity는 pulses per second (int32_t)
    int32_t velocity = ticks_per_sec;

    uint8_t node_id = node_ids[frame_index];

    frame.can_id = 0x100 | node_id;
    frame.can_dlc = 8;

    // 위치 (pulses, int32_t)
    frame.data[0] = (pos >> 24) & 0xFF;
    frame.data[1] = (pos >> 16) & 0xFF;
    frame.data[2] = (pos >> 8)  & 0xFF;
    frame.data[3] = pos & 0xFF;

    // 속도 (pulses per second, int32_t)
    frame.data[4] = (velocity >> 24) & 0xFF;
    frame.data[5] = (velocity >> 16) & 0xFF;
    frame.data[6] = (velocity >> 8)  & 0xFF;
    frame.data[7] = velocity & 0xFF;

    frame_index = (frame_index + 1) % num_nodes;

#if _LOG_
    std::cout << "[DummyCanDriver] Returning dummy frame for node 0x" 
              << std::hex << static_cast<int>(node_id) << std::dec 
              << " pos: " << pos << " pulses"
              << " velocity: " << velocity << " pulses/sec" << std::endl;
#endif

    return frame;
}
#endif

/*
//pulse_per_revolution = 10000 (1바퀴 당 10,000 펄스)
//wheel_radius = 0.05m → 바퀴 원둘레는 약 2 * π * 0.05 ≈ 0.314 m
//원하는 속도: desired_speed_mps = 0.3 m/s
    펄스 증가량 계산 과정
    한 펄스가 의미하는 실제 이동 거리:
    \text{pulse_distance} = \frac{\text{circumference}}{\text{pulse_per_revolution}} = \frac{0.314}{10000} = 0.0000314\, m
    초당 필요한 펄스 수:
    \text{ticks_per_sec} = \frac{\text{desired_speed_mps}}{\text{pulse_distance}} = \frac{0.3}{0.0000314} \approx 9554 \, \text{pulses/sec}
    node_update_hz = 100Hz / 4 노드 = 25 Hz → 각 노드는 초당 25번 업데이트됨
    프레임 당 펄스 증가량 (각 readFrame 호출 시 증가하는 펄스 수):
    \text{velocity} = \frac{\text{ticks_per_sec}}{\text{node_update_hz}} = \frac{9554}{25} \approx 382 \, \text{pulses/frame}
    
    바퀴 회전 수 변화
    positions[frame_index]는 누적 펄스 수입니다.

    한 바퀴 돌 때 10,000 펄스이므로,
    누적 펄스 수 10,000 = 바퀴 1바퀴 회전

    매 프레임마다 약 382 펄스가 더해지고
    약 26 프레임(100ms)마다 1바퀴의 약 1바퀴의 약 1/10 (382 * 26 ≈ 10,000) 누적되어
    초당 0.3 m/s 이동에 맞춰 바퀴가 회전하는 값이 됩니다.

    정리
    바퀴 회전수 (revolutions) = 누적 펄스 수 / pulse_per_revolution
    누적 펄스 수는 positions[frame_index]에 계속 쌓임
    이 값으로 실제 이동거리 = (누적 펄스 수 / pulse_per_revolution) * circumference

    pulse_per_revolution = 10000 기준으로 누적 펄스가 56만
    56만 펄스 ÷ 10000 펄스/회전 = 약 56 바퀴 회전

    바퀴 반지름 0.05m → 원둘레 약 0.314m
    56 바퀴 × 0.314m = 약 17.5 m 이동

    odom 위치 x가 17m 근처이므로,
    휠 엔코더 값에서 계산한 이동거리와 odom 위치 값이 정확하게 일치하는 것을 확인할 수 있습니다.
    
    정리
    휠 엔코더 펄스 값과 odom의 위치 값이 거의 일치하여 변환이 제대로 되고 있음
    velocity 값도 약 0.0004 m/s 정도로 천천히 꾸준히 증가하는 값이 보여짐

    이 값들은 설정한 더미 속도(desired_speed_mps = 0.3 m/s)보다 작지만, 프레임 갱신 주기나 기타 시뮬레이션 환경에 따라 조금씩 다를 수 있음
    휠 엔코더 펄스 → 거리 변환과 odom pose 갱신 로직이 정상적으로 동작하고 있다고 판단해도 됩니다.
    desired_speed_mps: 속도의 목표값 (단위: m/s)

    circumference: 휠 원둘레 = 2π * wheel_radius

    ticks_per_sec: 초당 펄스 수 = (속도) / (1펄스 거리) = desired_speed_mps / (circumference / pulse_per_revolution)

    velocity: 각 호출 프레임마다 증가할 pulse 수 = ticks_per_sec / node_update_hz

*/
#if 0
// 속도를 받아서 rpm, 위치(펄스) 업데이트
std::pair<int32_t, int32_t> DummyCanDriver::calcWheelState(
    double desired_speed_mps,
    int32_t& position,   // 누적 펄스 (in/out)
    double wheel_radius,
    int pulse_per_revolution,
    int node_update_hz
) {
    double circumference = 2.0 * M_PI * wheel_radius;  // m
    int ticks_per_sec = static_cast<int>(
        desired_speed_mps / (circumference / pulse_per_revolution)
    );

    double frame_time_sec = 1.0 / static_cast<double>(node_update_hz);

    // 누적 위치 업데이트
    position += static_cast<int32_t>(ticks_per_sec * frame_time_sec);

    // rpm 계산
    double rev_per_sec = static_cast<double>(ticks_per_sec) / pulse_per_revolution;
    int32_t rpm_int = static_cast<int32_t>(rev_per_sec * 60.0);

    return {position, rpm_int};
}
#endif
 
EncoderState DummyCanDriver::calcEncoderStateByDistance(double distance_moved, int32_t& position) {
    double pulses_per_meter = pulse_per_revolution_ / (2.0 * M_PI * wheel_radius_);
    int32_t delta_pulses = static_cast<int32_t>(distance_moved * pulses_per_meter);
    position += delta_pulses;

    // rpm 계산 (distance/time * 변환)
    double rev_per_sec = (distance_moved * pulses_per_meter) * control_hz_ / pulse_per_revolution_;
    int32_t rpm_int = static_cast<int32_t>(rev_per_sec * 60.0);

    return { position, rpm_int, static_cast<int>(distance_moved * pulses_per_meter * control_hz_) };
}

EncoderState DummyCanDriver::calcEncoderState(
        double desired_speed_mps,
        int32_t& position,
        double wheel_radius,
        int pulse_per_revolution,
        int node_update_hz
) {
    double circumference = 2.0 * M_PI * wheel_radius;  

    // 초당 펄스 수 (PPS)
    int pps = static_cast<int>(
        desired_speed_mps / (circumference / pulse_per_revolution)
    );

    double frame_time_sec = 1.0 / static_cast<double>(node_update_hz);

    // 누적 위치 업데이트
    position += static_cast<int32_t>(pps * frame_time_sec);

    // rpm 계산
    double rev_per_sec = static_cast<double>(pps) / pulse_per_revolution;
    int32_t rpm_int = static_cast<int32_t>(rev_per_sec * 60.0);

    return { position, rpm_int, pps };
}

#if 0
::can_frame DummyCanDriver::readFrame() {
    std::lock_guard<std::mutex> lock(mutex_);
    ::can_frame frame{};

    if (!is_open_) {
        std::cout << "[DummyCanDriver] readFrame failed: interface not open" << std::endl;
        return frame;
    }

    static int frame_index = 0;
    double dt = 1.0 / static_cast<double>(control_hz_ / num_nodes_);

    // 사각형 경로 위치 업데이트 (속도 * dt)
    double move_x = 0.0, move_y = 0.0;
    switch(step_) {
        case 0: move_x = speed_mps_ * dt; break;
        case 1: move_y = -speed_mps_ * dt; break;
        case 2: move_x = -speed_mps_ * dt; break;
        case 3: move_y = speed_mps_ * dt; break;
    }

    current_position_.x += move_x;
    current_position_.y += move_y;

    // 다음 단계 전환
    if (step_ == 0 && current_position_.x >= side_length_) step_ = 1;
    else if (step_ == 1 && current_position_.y <= -side_length_) step_ = 2;
    else if (step_ == 2 && current_position_.x <= 0.0) step_ = 3;
    else if (step_ == 3 && current_position_.y >= 0.0) step_ = 0;

    // 이동 거리
    double distance_moved = std::sqrt(move_x * move_x + move_y * move_y);

    // 누적 펄스 및 rpm 계산
    EncoderState ws = calcEncoderStateByDistance(distance_moved, positions_[frame_index]);

    uint8_t node_id = 0x01 + frame_index;
    frame.can_id = 0x100 | node_id;
    frame.can_dlc = 8;

    // 위치 펄스 (position)
    frame.data[0] = (ws.position >> 24) & 0xFF;
    frame.data[1] = (ws.position >> 16) & 0xFF;
    frame.data[2] = (ws.position >> 8) & 0xFF;
    frame.data[3] = ws.position & 0xFF;

    // 속도 rpm
    frame.data[4] = (ws.rpm >> 24) & 0xFF;
    frame.data[5] = (ws.rpm >> 16) & 0xFF;
    frame.data[6] = (ws.rpm >> 8) & 0xFF;
    frame.data[7] = ws.rpm & 0xFF;

    frame_index = (frame_index + 1) % num_nodes_;
    return frame;
}
#else
::can_frame DummyCanDriver::readFrame() {
    std::lock_guard<std::mutex> lock(mutex_);
    ::can_frame frame{};

    if (!is_open_) {
        std::cout << "[DummyCanDriver] readFrame failed: interface not open" << std::endl;
        return frame;
    }

    static int frame_index = 0;
    const uint8_t node_ids[4] = {0x01, 0x02, 0x03, 0x04};
    static int32_t positions[4] = {0, 0, 0, 0};

    const int control_hz = 100;
    const int num_nodes = 4;
    const int node_update_hz = control_hz / num_nodes;

    const double wheel_radius = 0.05;
    const int pulse_per_revolution = 10000;

#ifndef DUMMY_SPEED_MPS
#define DUMMY_SPEED_MPS 0.6
#endif
    const double desired_speed_mps = DUMMY_SPEED_MPS;

    EncoderState ws = calcEncoderState(
        desired_speed_mps,
        positions[frame_index],
        wheel_radius,
        pulse_per_revolution,
        node_update_hz
    );

    uint8_t node_id = node_ids[frame_index];
    frame.can_id = 0x100 | node_id;
    frame.can_dlc = 8;

    // 위치 (position, pulses)
    frame.data[0] = (ws.position >> 24) & 0xFF;
    frame.data[1] = (ws.position >> 16) & 0xFF;
    frame.data[2] = (ws.position >> 8)  & 0xFF;
    frame.data[3] = ws.position & 0xFF;

#ifndef SET_RPM
#define SET_RPM 1
#endif
#if SET_RPM
    // 속도 (rpm)
    frame.data[4] = (ws.rpm >> 24) & 0xFF;
    frame.data[5] = (ws.rpm >> 16) & 0xFF;
    frame.data[6] = (ws.rpm >> 8)  & 0xFF;
    frame.data[7] = ws.rpm & 0xFF;
#else
    // PPS (pulse per second)  
    frame.data[4] = (ws.pps >> 24) & 0xFF;
    frame.data[5] = (ws.pps >> 16) & 0xFF;
    frame.data[6] = (ws.pps >> 8)  & 0xFF;
    frame.data[7] = ws.pps & 0xFF;
#endif
#if _LOG_
    std::cout << "[DummyCanDriver] node 0x"
              << std::hex << static_cast<int>(node_id) << std::dec
              << " pos: " << ws.position << " pulses"
              << " pps: " << ws.pps
              << std::endl;
#endif

    frame_index = (frame_index + 1) % num_nodes;
    return frame;
}
#endif

//PEAK
TPCANMsg_ DummyCanDriver::readPeakFrame() {
    TPCANMsg_ empty_msg{};
    empty_msg.ID = 0;
    empty_msg.MSGTYPE = 0;
    empty_msg.LEN = 0;
    for (int i = 0; i < 8; ++i)
        empty_msg.DATA[i] = 0;
    return empty_msg;
}

} // namespace Inabot
