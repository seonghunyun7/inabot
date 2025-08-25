#pragma once

#include "can_types.hpp"
#include "motor/can_driver/i_can_interface.hpp"
#include <vector>
#include <mutex>

namespace Inabot {

class PeakCanDriver : public ICanInterface {
public:
    PeakCanDriver();
    virtual ~PeakCanDriver();

    bool initialize(const std::string& interface_name) override;
    void close() override;
    bool sendFrame(uint32_t can_id, const std::vector<uint8_t>& data) override;

    /**
     * NOTE: SocketCAN의 `can_frame`을 제거하므로,
     * `readFrame`을 인터페이스 요구에 맞게 더미로 구현하거나 
     * 내부 포맷(TPCANMsg_)로 대체할 수 있음
     */
    // ICanInterface 의 가상 함수 override
    TPCANMsg_ readPeakFrame() override;

    // ICanInterface 요구사항 때문에 구현 필요
    ::can_frame readFrame() override;

private:
    bool is_open_;
    std::mutex mutex_;
};

} // namespace Inabot

