#pragma once
#include <string>
#include <functional>
#include <thread>
#include <atomic>

class HeartbeatManager {
public:
    using PublishFunc = std::function<void(const std::string&, const std::string&)>;

    HeartbeatManager(const std::string& manufacturer,
                     const std::string& serial_number,
                     PublishFunc pub_func,
                     int interval_sec = 15)
        : manufacturer_(manufacturer),
          serial_number_(serial_number),
          publish_func_(pub_func),
          interval_(interval_sec),
          running_(false) {}

    ~HeartbeatManager() { stop(); }

    void start();
    void stop();
    void sendHeartbeat(const std::string& state);

private:
    std::string manufacturer_;
    std::string serial_number_;
    PublishFunc publish_func_;
    int interval_;
    std::thread thread_;
    std::atomic<bool> running_;
};
