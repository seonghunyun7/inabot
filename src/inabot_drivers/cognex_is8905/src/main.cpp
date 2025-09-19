#include "rclcpp/rclcpp.hpp"
#include "cognex_is8905.hpp"
#include <csignal>
#include <memory>
#include <vector>

std::vector<std::shared_ptr<Cognex_IS8905>> cognex_nodes;

void signal_handler(int signal)
{
    if (signal == SIGINT || signal == SIGTERM) {
        RCLCPP_INFO(rclcpp::get_logger("cognex_is8905_node"), "Signal received, shutting down...");
        for (auto & node : cognex_nodes) {
            node->stop();
        }
        rclcpp::shutdown();
    }
}

int main(int argc, char ** argv)
{
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    options.use_intra_process_comms(true);

    auto node = std::make_shared<Cognex_IS8905>(options);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}