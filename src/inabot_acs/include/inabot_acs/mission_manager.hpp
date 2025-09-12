#pragma once
#include <memory>
#include <string>
#include <nlohmann/json.hpp>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class MissionManager {
public:
    using DisconnectCallback = std::function<void()>;

    MissionManager(std::shared_ptr<rclcpp::Node> node);

    // VD5050D 전용 메시지 핸들러
    void handleOrder(const nlohmann::json& payload);
    void handleInstantActions(const nlohmann::json& payload);

    // TEST 메시지 핸들러
    void handleMission(const nlohmann::json& payload);
    void handleControl(const nlohmann::json& payload);
    void handleHeartbeat(const nlohmann::json& payload);

    // 연결 해제 콜백 등록
    void setDisconnectCallback(DisconnectCallback cb);

private:
    std::shared_ptr<rclcpp::Node> node_;
    // VD5050D 퍼블리셔
    // FMS => ROBOT
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr order_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr instant_actions_pub_;

    // FOR_TEST
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mission_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr control_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr heartbeat_pub_;

    DisconnectCallback disconnect_cb_;
};
