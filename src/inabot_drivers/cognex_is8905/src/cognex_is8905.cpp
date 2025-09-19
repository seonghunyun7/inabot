#include "cognex_is8905.hpp"
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <cerrno>

Cognex_IS8905::Cognex_IS8905(const rclcpp::NodeOptions & options)
: Node("cognex_is8905_node", options), running_(true)
{
    publisher_ = this->create_publisher<std_msgs::msg::Int32>("cognex_is8905_data", 10); // 0,1,2 ...

    this->declare_parameter<std::string>("ip_address", "192.168.1.50");
    this->declare_parameter<int>("port", 5000);
    this->get_parameter("ip_address", ip_address_);
    this->get_parameter("port", port_);

    socket_thread_ = std::thread(&Cognex_IS8905::socketThreadFunc, this);
}

Cognex_IS8905::~Cognex_IS8905()
{
    stop();
    if (socket_thread_.joinable())
        socket_thread_.join();
}

void Cognex_IS8905::stop()
{
    running_ = false;
    if (sock_ >= 0) {
        close(sock_);
        sock_ = -1;
    }
}

void Cognex_IS8905::socketThreadFunc()
{
    sock_ = socket(AF_INET, SOCK_STREAM, 0);
    if (sock_ < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to create socket");
        return;
    }

    // 1초 read timeout
    struct timeval tv;
    tv.tv_sec = 1;
    tv.tv_usec = 0;
    setsockopt(sock_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    sockaddr_in server_addr{};
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(port_);

    if (inet_pton(AF_INET, ip_address_.c_str(), &server_addr.sin_addr) <= 0) {
        RCLCPP_ERROR(this->get_logger(), "Invalid IP address: %s", ip_address_.c_str());
        close(sock_);
        sock_ = -1;
        return;
    }

    if (connect(sock_, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to connect to %s:%d", ip_address_.c_str(), port_);
        close(sock_);
        sock_ = -1;
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Connected to Cognex IS8905 at %s:%d", ip_address_.c_str(), port_);

    char buffer[1024];
    while (running_) {
        memset(buffer, 0, sizeof(buffer));
        int val = read(sock_, buffer, sizeof(buffer) - 1);

        if (val > 0) {
            try {
                int data = std::stoi(buffer);
                auto msg = std_msgs::msg::Int32();
                msg.data = data;
                publisher_->publish(msg);
#if __LOG__
                RCLCPP_INFO(this->get_logger(), "Received data: %d", data);
#endif
            } catch (...) {
                RCLCPP_WARN(this->get_logger(), "Failed to parse data: '%s'", buffer);
            }
        } else if (val == 0) {
            RCLCPP_WARN(this->get_logger(), "Connection closed by camera");
            break;
        } else if (errno == EWOULDBLOCK || errno == EAGAIN) {
            // 타임아웃 시 짧은 sleep으로 CPU 부하 완화
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        } else {
            RCLCPP_WARN(this->get_logger(), "Read error");
            break;
        }
    }

    if (sock_ >= 0) {
        close(sock_);
        sock_ = -1;
    }
}