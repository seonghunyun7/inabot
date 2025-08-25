#pragma once

#include "i_can_interface.hpp"
#include <vector>
#include <mutex>
#include <linux/can.h>  // ::can_frame 필요
#include "can_types.hpp" // TPCANMsg_ 타입 정의

struct Position2D {
    double x = 0.0;
    double y = 0.0;
};

struct EncoderState {
    int32_t position; // 누적 펄스
    int32_t rpm;      // RPM
    int32_t pps;      // 초당 펄스 수
};

namespace Inabot {

class DummyCanDriver : public ICanInterface {
public:
    DummyCanDriver();
    virtual ~DummyCanDriver();

    bool initialize(const std::string& interface_name) override;
    void close() override;

    bool sendFrame(uint32_t can_id, const std::vector<uint8_t>& data) override;
    ::can_frame readFrame() override;

    // ICanInterface에 순수 가상 함수로 있다면 반드시 구현해야 함
    TPCANMsg_ readPeakFrame() override;

private:

    bool is_open_;
    std::mutex mutex_;

    Position2D current_position_{0.0, 0.0};
    int step_ = 0;  // 0=오른쪽,1=아래,2=왼쪽,3=위
    const double side_length_ = 10.0;
    const double speed_mps_ = 0.5;  // 0.5m/s 이동 속도
    const double wheel_radius_ = 0.05;
    const int pulse_per_revolution_ = 10000;
    const int control_hz_ = 100; // 호출 주기 (Hz)
    const int num_nodes_ = 4;

    int32_t positions_[4] = {0,0,0,0};  // 각 노드별 누적 펄스

    EncoderState calcEncoderStateByDistance(double distance_moved, int32_t& position);

    EncoderState calcEncoderState(
        double desired_speed_mps,
        int32_t& position,
        double wheel_radius,
        int pulse_per_revolution,
        int node_update_hz
    );

};

} // namespace Inabot
