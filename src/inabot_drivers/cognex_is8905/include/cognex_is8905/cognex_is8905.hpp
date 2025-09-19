#pragma once

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include <thread>
#include <atomic>
#include <string>

class Cognex_IS8905 : public rclcpp::Node
{
public:
    Cognex_IS8905(const rclcpp::NodeOptions & options);
    ~Cognex_IS8905();

    void stop();

private:
    void socketThreadFunc();
    void initConnection();

    std::thread socket_thread_;
    std::atomic<bool> running_;
    int sock_{-1};

    std::string ip_address_;
    int port_{5000};

    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
};
