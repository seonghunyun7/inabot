#include <csignal>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "ota_node.hpp"

std::shared_ptr<OTANode> g_node = nullptr;

void signalHandler(int signal)
{
    if (g_node) {
        RCLCPP_INFO(g_node->get_logger(), "Signal %d received, shutting down...", signal);
        g_node->stopServer();
    }
    rclcpp::shutdown();
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    g_node = std::make_shared<OTANode>();

    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    rclcpp::spin(g_node);
    rclcpp::shutdown();

    return 0;
}
