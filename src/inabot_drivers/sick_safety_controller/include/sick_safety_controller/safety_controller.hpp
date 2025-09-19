#pragma once

#include "rclcpp/rclcpp.hpp"
#include "inabot_msgs/msg/plc_status.hpp"
#include "inabot_msgs/msg/plc_control.hpp"
#include <modbus/modbus.h>
#include <vector>
#include <mutex>

#define TYPE_CHARGE 1

class SafetyController : public rclcpp::Node
{
public:
    SafetyController();
    ~SafetyController();

    void reset();
    std::vector<std::vector<bool>> readInputs(); // 입력/출력 상태
    void writeOutput(int coil_type, bool value); // 출력 제어

private:
    void timerCallback();
    void controlCallback(const inabot_msgs::msg::PLCControl::SharedPtr msg);

    rclcpp::Publisher<inabot_msgs::msg::PLCStatus>::SharedPtr status_pub_;
    rclcpp::Subscription<inabot_msgs::msg::PLCControl>::SharedPtr control_sub_;

    rclcpp::TimerBase::SharedPtr timer_;
    modbus_t* ctx_;
    std::mutex lock_;
};
