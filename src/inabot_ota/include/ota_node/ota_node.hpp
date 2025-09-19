// ota_node.hpp
#ifndef OTA_NODE_HPP
#define OTA_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include <string>
#include <thread>
#include <filesystem>

class OTANode : public rclcpp::Node
{
public:
    OTANode();
    ~OTANode();

    void startServer();
    void deployUpdate();
    void stopServer();
    void prepareForOTA();
    void sendStatusToClient(const std::string &status);

private:
    void publishStatus(const std::string &msg);

    std::thread server_thread_;
    bool server_running_;
    int server_fd_;
    int client_socket_fd_{-1};

    std::string TEMP_ARCHIVE_;
    std::string OTA_HOST_;
    int OTA_PORT_;
    std::string INSTALL_DIR_;
    std::string BACKUP_DIR_BASE_;
};

#endif // OTA_NODE_HPP
