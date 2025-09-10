#pragma once
#include <memory>
#include <string>
#include <nlohmann/json.hpp>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class MissionManager {
public:
    MissionManager(std::shared_ptr<rclcpp::Node> node);

    void handleMission(const nlohmann::json& payload);
    void handleControl(const nlohmann::json& payload);
    void handleHeartbeat(const nlohmann::json& payload);

    // VD5050D 전용 메시지 핸들러
    void handleOrder(const nlohmann::json& payload);
    void handleInstantActions(const nlohmann::json& payload);
    void handleState(const nlohmann::json& payload);
    void handleVisualization(const nlohmann::json& payload);
    void handleConnection(const nlohmann::json& payload);
    void handleFactsheet(const nlohmann::json& payload);

private:
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mission_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr control_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr heartbeat_pub_;

    // VD5050D 퍼블리셔
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr order_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr instant_actions_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr visualization_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr connection_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr factsheet_pub_;
};
