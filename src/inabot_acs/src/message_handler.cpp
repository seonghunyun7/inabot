// message_handler.cpp
#include "inabot_acs/topics.hpp"
#include "message_handler.hpp"
#include "rclcpp/rclcpp.hpp"
#include <nlohmann/json.hpp>
#include <iostream>
#include <unordered_map>
#include <functional>

MessageHandler::MessageHandler(std::shared_ptr<MissionManager> mission_mgr)
: mission_manager_(mission_mgr)
{
    // 토픽 → 처리 함수 매핑
    handlers_ = {
        #if _FOR_TEST
        {inabot_acs::TOPIC_MISSION,      [this](const nlohmann::json& j){ mission_manager_->handleMission(j); }},
        {inabot_acs::TOPIC_CONTROL,      [this](const nlohmann::json& j){ mission_manager_->handleControl(j); }},
        {inabot_acs::TOPIC_HEARTBEAT,    [this](const nlohmann::json& j){ mission_manager_->handleHeartbeat(j); }},
        #endif // fms -> robot        
        {inabot_acs::TOPIC_ORDER,        [this](const nlohmann::json& j){ mission_manager_->handleOrder(j); }},           // VD5050D
        {inabot_acs::TOPIC_INSTANT_ACTIONS, [this](const nlohmann::json& j){ mission_manager_->handleInstantActions(j); }}, // VD5050D
    };
}

void MessageHandler::handleMessage(const std::string& topic, const std::string& payload)
{
    try {
        auto json = nlohmann::json::parse(payload);
        std::cout << "[msg_handler] Received topic: " << topic 
                  << ", payload:\n" << json.dump(4) << std::endl;

        auto it = handlers_.find(topic);
        if(it != handlers_.end()) {
            it->second(json); // 매핑된 처리 함수 호출
        } else {
            std::cout << "[msg_handler] Warning: Unknown topic: " << topic << std::endl;
        }

    } catch(const nlohmann::json::parse_error& e) {
        std::cerr << "[msg_handler] JSON parse error: " << e.what() << std::endl;
    }
}
