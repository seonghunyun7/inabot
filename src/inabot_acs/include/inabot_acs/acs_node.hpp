#pragma once
#include "rclcpp/rclcpp.hpp"
#include "mqtt_client.hpp"
#include "mission_manager.hpp"
#include "message_handler.hpp"

// acs_node.hpp
class AcsNode : public rclcpp::Node {
public:
    AcsNode();
    ~AcsNode();

    void init();

private:
    std::shared_ptr<MissionManager> mission_manager_;
    std::shared_ptr<MessageHandler> msg_handler_;
    std::shared_ptr<MqttClient> mqtt_client_;
};