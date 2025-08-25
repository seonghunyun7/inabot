#ifndef MONITOR_NODE_HPP
#define MONITOR_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <string>

#include "monitor/system_monitor.hpp"
#include "monitor/network_monitor.hpp"
#include "monitor/device_topic_monitor.hpp"

class MonitorNode : public rclcpp::Node
{
public:
    MonitorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:

    void initParameters();
    void timerCallback();

    std::unique_ptr<SystemMonitor> system_monitor_;
    std::unique_ptr<NetworkMonitor> network_monitor_;
    std::unique_ptr<DeviceTopicMonitor> device_topic_monitor_;

    std::string bridge_router_ip_;
    std::string factory_router_ip_;
    std::string disk_device_;

    bool use_network_monitor_;
    bool use_system_monitor_;
 
    rclcpp::TimerBase::SharedPtr timer_;
};

#endif // MONITOR_NODE_HPP
