#pragma once

#include "i_can_interface.hpp"
#include <mutex>
#include <string>
#include <linux/can.h>  // ::can_frame 필요
#include "can_types.hpp" // TPCANMsg_ 타입 정의

namespace Inabot {

class SocketCanDriver : public ICanInterface {
public:
    SocketCanDriver();
    virtual ~SocketCanDriver();

    bool initialize(const std::string& interface_name) override;
    void close() override;

    bool sendFrame(uint32_t can_id, const std::vector<uint8_t>& data) override;
    ::can_frame readFrame() override;  // 전역 네임스페이스 사용

    // ICanInterface에 순수 가상 함수로 있다면 반드시 구현해야 함
    TPCANMsg_ readPeakFrame() override;

private:
    bool checkBitrateStatus(const std::string& interface_name);

    int socket_fd_;
    bool is_open_;
    std::string interface_name_;
    std::mutex socket_mutex_;
};

} // namespace Inabot
