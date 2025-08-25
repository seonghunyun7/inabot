#pragma once

#include <vector>
#include <cstdint>
#include <string>
#include <linux/can.h>  // struct can_frame 정의
#include "can_types.hpp"  // TPCANMsg_ 타입 포함

namespace Inabot {

class ICanInterface {
public:
    virtual ~ICanInterface() = default;

    // 인터페이스 초기화
    virtual bool initialize(const std::string& interface_name) = 0;

    // 인터페이스 닫기
    virtual void close() = 0;

    // CAN 프레임 전송
    virtual bool sendFrame(uint32_t can_id, const std::vector<uint8_t>& data) = 0;

    // CAN 프레임 수신
    virtual ::can_frame readFrame() = 0;  // 전역 네임스페이스 명시
    
    // === PEAK CAN 전용 수신 함수 ===
    virtual TPCANMsg_ readPeakFrame() = 0;
};

} // namespace Inabot
