#include "heartbeat_manager.hpp"
#include <nlohmann/json.hpp>
#include <chrono>
#include <thread>
#include <iostream>

void HeartbeatManager::start() {
    running_ = true;
    thread_ = std::thread([this]() {
        while (running_) {
            sendHeartbeat("ONLINE");
            std::this_thread::sleep_for(std::chrono::seconds(interval_));
        }
    });
}

void HeartbeatManager::stop() {
    running_ = false;
    if (thread_.joinable()) thread_.join();
    sendHeartbeat("OFFLINE");
}

void HeartbeatManager::sendHeartbeat(const std::string& state) {
    nlohmann::json msg;
    msg["headerId"] = 0;

    //"2025-09-11T10:00:00Z"
    auto now = std::chrono::system_clock::now();
    std::time_t t_c = std::chrono::system_clock::to_time_t(now);
    char buf[64];
    std::strftime(buf, sizeof(buf), "%FT%TZ", std::gmtime(&t_c)); 
    msg["timestamp"] = buf;
    msg["version"] = "2.0.0";
    msg["manufacturer"] = manufacturer_;
    msg["serialNumber"] = serial_number_;
    msg["connectionState"] = state;

    std::string topic = "uagv/v2/" + manufacturer_ + "/" + serial_number_ + "/connection";

    if (publish_func_) {
        publish_func_(topic, msg.dump());
        #if __LOG__
        std::cout << "[Heartbeat] Sent " << state << " to " << topic << std::endl;
        #endif
    }
}
