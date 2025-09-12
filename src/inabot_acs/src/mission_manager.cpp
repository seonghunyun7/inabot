// mission_manager.cpp
#include "mission_manager.hpp"
#include <iostream>

MissionManager::MissionManager(std::shared_ptr<rclcpp::Node> node)
: node_(node)
{
    // 기존 퍼블리셔
    #if __FOR_TEST
    mission_pub_ = node_->create_publisher<std_msgs::msg::String>("mission_topic", 10);
    control_pub_ = node_->create_publisher<std_msgs::msg::String>("control_topic", 10);
    heartbeat_pub_ = node_->create_publisher<std_msgs::msg::String>("heartbeat_topic", 10);
    #endif
    // VD5050D 퍼블리셔
    order_pub_ = node_->create_publisher<std_msgs::msg::String>("order_topic", 10);
    instant_actions_pub_ = node_->create_publisher<std_msgs::msg::String>("instant_actions_topic", 10);

    std::cout << "[mission_manager] Publishers created for mission, control, heartbeat and VD5050D topics" << std::endl;
}

void MissionManager::setDisconnectCallback(DisconnectCallback cb)
{
    disconnect_cb_ = cb;
}

// VD5050D 핸들러
void MissionManager::handleOrder(const nlohmann::json& payload)
{
    std::cout << "[mission_manager] Handling order:\n" << payload.dump(4) << std::endl;
    std_msgs::msg::String msg; msg.data = payload.dump();
    order_pub_->publish(msg);
}

void MissionManager::handleInstantActions(const nlohmann::json& payload)
{
    std::cout << "[mission_manager] Handling instantActions:\n" << payload.dump(4) << std::endl;
    std_msgs::msg::String msg; msg.data = payload.dump();
    instant_actions_pub_->publish(msg);
}

#if 0 // roboot -> fms
void MissionManager::handleConnection(const nlohmann::json& payload)
{
    std::cout << "[mission_manager] Handling connection:\n" 
              << payload.dump(4) << std::endl;

    std_msgs::msg::String msg; 
    msg.data = payload.dump();
    connection_pub_->publish(msg);

    // connectionState 확인
    if (payload.contains("connectionState")) {
        std::string state = payload["connectionState"];

        if (state == "OFFLINE" || state == "CONNECTIONBROKEN") {
            std::cout << "[mission_manager] Connection lost: " << state << std::endl;
            if (disconnect_cb_) {
                disconnect_cb_();
            }
        }
    }
}
#endif

// TEST 핸들러
void MissionManager::handleMission(const nlohmann::json& payload)
{
    std_msgs::msg::String msg; msg.data = payload.dump();
    mission_pub_->publish(msg);
}

void MissionManager::handleControl(const nlohmann::json& payload)
{
    std_msgs::msg::String msg; msg.data = payload.dump();
    control_pub_->publish(msg);
}

void MissionManager::handleHeartbeat(const nlohmann::json& payload)
{
    std_msgs::msg::String msg; msg.data = payload.dump();
    heartbeat_pub_->publish(msg);
}
