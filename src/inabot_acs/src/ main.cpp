#include "rclcpp/rclcpp.hpp"
#include "acs_node.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // shared_ptr로 노드 생성
    auto node = std::make_shared<AcsNode>();
    node->init();

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
