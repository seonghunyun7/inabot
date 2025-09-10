// message_handler.hpp
#pragma once
#include <string>
#include <unordered_map>
#include <functional>
#include <nlohmann/json.hpp>
#include <memory>
#include "mission_manager.hpp"

class MessageHandler {
public:
    MessageHandler(std::shared_ptr<MissionManager> mission_mgr);
    void handleMessage(const std::string& topic, const std::string& payload);

private:
    std::shared_ptr<MissionManager> mission_manager_;
    std::unordered_map<std::string, std::function<void(const nlohmann::json&)>> handlers_;
};
