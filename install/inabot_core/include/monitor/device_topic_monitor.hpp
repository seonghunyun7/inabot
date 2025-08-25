#ifndef DEVICE_TOPIC_MONITOR_HPP
#define DEVICE_TOPIC_MONITOR_HPP

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
//bms_status.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class DeviceTopicMonitor
{
public:
    DeviceTopicMonitor(rclcpp::Node* node, rclcpp::Logger logger = rclcpp::get_logger("DeviceTopicMonitor"));

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    #if _USED_
    void bmsBatteryCallback(const Inabot_msgs::msg::BmsStatus::SharedPtr msg);
    #endif
    void encoderCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void scanFrontCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void scanRearCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

    rclcpp::Node* node_;
    rclcpp::Logger logger_;
    std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
};

#endif // DEVICE_TOPIC_MONITOR_HPP
