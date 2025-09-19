#include "rclcpp/rclcpp.hpp"
#include "sound.hpp"
#include <csignal>

std::shared_ptr<Sound> g_sound_node = nullptr;

void signal_handler(int signum)
{
    if (signum == SIGINT || signum == SIGTERM) {
        RCLCPP_INFO(rclcpp::get_logger("sound_driver"), "Signal received, shutting down...");
        if (g_sound_node) {
            g_sound_node->shutdown_sound();  // 시그널 시 ffplay 종료
        }
        rclcpp::shutdown();
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    g_sound_node = std::make_shared<Sound>();
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    rclcpp::spin(g_sound_node);
    rclcpp::shutdown();
    return 0;
}