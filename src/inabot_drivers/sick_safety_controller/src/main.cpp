#include "rclcpp/rclcpp.hpp"
#include "sick_safety_controller/safety_controller.hpp"
#include <memory>
#include <csignal>

std::shared_ptr<SafetyController> g_node = nullptr;

void signal_handler(int signum)
{
    if (signum == SIGINT || signum == SIGTERM) {
        if (g_node) {
            RCLCPP_INFO(g_node->get_logger(), "Signal received, shutting down Modbus and node...");
            g_node->reset();
        }
        rclcpp::shutdown();
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    g_node = std::make_shared<SafetyController>();

    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    rclcpp::spin(g_node);
    rclcpp::shutdown();

    return 0;
}
