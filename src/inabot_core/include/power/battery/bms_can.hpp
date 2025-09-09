#pragma once

#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <cstring>
#include <sys/types.h>
#include <sys/socket.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <unistd.h>
#include <sys/ioctl.h>

#include "inabot_msgs/msg/bms_state.hpp"
#include "inabot_msgs/msg/bms_can_frame.hpp"

class CBmsCan : public rclcpp::Node
{
public:
    explicit CBmsCan(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~CBmsCan();

private:
    void loadParameters();
    void connectCan();
    void readCan();
    void publishState();
    bool parseMessage(uint32_t msg_id, const uint8_t* data, size_t len);

    // 두 바이트를 Big Endian 형식으로 읽어 int16_t로 변환
    int16_t toInt16BE(const uint8_t* d);
    // 두 바이트를 Big Endian 형식으로 읽어 uint16_t로 변환
    uint16_t toUInt16BE(const uint8_t* d);

    // 테스트용 CAN 프레임 콜백
    void testCanFrameCallback(const inabot_msgs::msg::BmsCanFrame::SharedPtr msg);

private:
    int can_socket_;
    std::string can_interface_;

    rclcpp::Publisher<inabot_msgs::msg::BmsState>::SharedPtr batt_pub_;
    rclcpp::Subscription<inabot_msgs::msg::BmsCanFrame>::SharedPtr test_can_frame_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    inabot_msgs::msg::BmsState current_state_; // << 누적 상태
};
