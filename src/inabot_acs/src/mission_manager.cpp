// mission_manager.cpp
#include "mission_manager.hpp"
#include <iostream>

MissionManager::MissionManager(std::shared_ptr<rclcpp::Node> node)
: node_(node)
{
    // 기존 퍼블리셔
    mission_pub_ = node_->create_publisher<std_msgs::msg::String>("mission_topic", 10);
    control_pub_ = node_->create_publisher<std_msgs::msg::String>("control_topic", 10);
    heartbeat_pub_ = node_->create_publisher<std_msgs::msg::String>("heartbeat_topic", 10);

    // VD5050D 퍼블리셔
    order_pub_ = node_->create_publisher<std_msgs::msg::String>("order_topic", 10);
    instant_actions_pub_ = node_->create_publisher<std_msgs::msg::String>("instant_actions_topic", 10);
    state_pub_ = node_->create_publisher<std_msgs::msg::String>("state_topic", 10);
    visualization_pub_ = node_->create_publisher<std_msgs::msg::String>("visualization_topic", 10);
    connection_pub_ = node_->create_publisher<std_msgs::msg::String>("connection_topic", 10);
    factsheet_pub_ = node_->create_publisher<std_msgs::msg::String>("factsheet_topic", 10);

    std::cout << "[mission_manager] Publishers created for mission, control, heartbeat and VD5050D topics" << std::endl;
}

// 기존 핸들러
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

void MissionManager::handleState(const nlohmann::json& payload)
{
    std::cout << "[mission_manager] Handling state:\n" << payload.dump(4) << std::endl;
    std_msgs::msg::String msg; msg.data = payload.dump();
    state_pub_->publish(msg);
}

void MissionManager::handleVisualization(const nlohmann::json& payload)
{
    std::cout << "[mission_manager] Handling visualization:\n" << payload.dump(4) << std::endl;
    std_msgs::msg::String msg; msg.data = payload.dump();
    visualization_pub_->publish(msg);
}

void MissionManager::handleConnection(const nlohmann::json& payload)
{
    std::cout << "[mission_manager] Handling connection:\n" << payload.dump(4) << std::endl;
    std_msgs::msg::String msg; msg.data = payload.dump();
    connection_pub_->publish(msg);
}

void MissionManager::handleFactsheet(const nlohmann::json& payload)
{
    std::cout << "[mission_manager] Handling factsheet:\n" << payload.dump(4) << std::endl;
    std_msgs::msg::String msg; msg.data = payload.dump();
    factsheet_pub_->publish(msg);
}
