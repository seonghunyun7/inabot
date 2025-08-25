#include "monitor/monitor.hpp"
#include <iostream>

MonitorNode::MonitorNode(const rclcpp::NodeOptions & options)
: Node("monitor_node", options)
{
    initParameters();
    
    if ( use_system_monitor_)
        system_monitor_ = std::make_unique<SystemMonitor>(disk_device_);

    if ( use_network_monitor_)
        network_monitor_ = std::make_unique<NetworkMonitor>(bridge_router_ip_, factory_router_ip_);

    device_topic_monitor_ = std::make_unique<DeviceTopicMonitor>(this, this->get_logger());

    timer_ = this->create_wall_timer(
        std::chrono::seconds(10),
        std::bind(&MonitorNode::timerCallback, this));
}

void MonitorNode::initParameters() 
{
    this->declare_parameter<std::string>("bridge_router_ip", "211.195.215.1");
    this->declare_parameter<std::string>("factory_router_ip", "8.8.8.8");
    this->declare_parameter<std::string>("disk_device", "nvme0n1");
    this->declare_parameter<bool>("use_network_monitor", false);
    this->declare_parameter<bool>("use_system_monitor", false);

    this->get_parameter("bridge_router_ip", bridge_router_ip_);
    this->get_parameter("factory_router_ip", factory_router_ip_);
    this->get_parameter("disk_device", disk_device_);
    this->get_parameter("bridge_router_ip", bridge_router_ip_);
    this->get_parameter("use_network_monitor", use_network_monitor_);
    this->get_parameter("use_system_monitor", use_system_monitor_);

    RCLCPP_INFO(this->get_logger(), "bridge_router_ip_: %s", bridge_router_ip_.c_str());
    RCLCPP_INFO(this->get_logger(), "factory_router_ip_: %s", factory_router_ip_.c_str());
    RCLCPP_INFO(this->get_logger(), "disk_device_: %s", disk_device_.c_str());    
}

void MonitorNode::timerCallback()
{
    if ( use_system_monitor_)
        system_monitor_->getSystemStatus();

    if ( use_network_monitor_)
        network_monitor_->checkNetworkStatus();
}
