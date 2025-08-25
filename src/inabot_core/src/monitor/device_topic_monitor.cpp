#include "monitor/device_topic_monitor.hpp"

DeviceTopicMonitor::DeviceTopicMonitor(rclcpp::Node* node, rclcpp::Logger logger)
: node_(node), logger_(logger)
{
    auto sensor_qos = rclcpp::SensorDataQoS().keep_last(10);

    subscriptions_.push_back(
        node_->create_subscription<nav_msgs::msg::Odometry>(
            "odom",
            /*rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile()*/10,
            std::bind(&DeviceTopicMonitor::odomCallback, this, std::placeholders::_1)
        )
    );

    #if _USED_
    subscriptions_.push_back(
        node_->create_subscription<Inabot_msgs::msg::BmsStatus>(
            "bms_status",
            rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile(),
            std::bind(&DeviceTopicMonitor::bmsBatteryCallback, this, std::placeholders::_1)
        )
    );
    #endif

    //wheel-encoder
    subscriptions_.push_back(
        node_->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states",
            /*sensor_qos*/ 10,
            std::bind(&DeviceTopicMonitor::encoderCallback, this, std::placeholders::_1)
        )
    );

    subscriptions_.push_back(
        node_->create_subscription<sensor_msgs::msg::Imu>(
            "/imu",
            /*sensor_qos*/ 10,
            std::bind(&DeviceTopicMonitor::imuCallback, this, std::placeholders::_1)
        )
    );

    subscriptions_.push_back(
        node_->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan_front",
            /*sensor_qos*/ 10,
            std::bind(&DeviceTopicMonitor::scanFrontCallback, this, std::placeholders::_1)
        )
    );

    subscriptions_.push_back(
        node_->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan_rear",
            /*sensor_qos*/ 10,
            std::bind(&DeviceTopicMonitor::scanRearCallback, this, std::placeholders::_1)
        )
    );

    subscriptions_.push_back(
        node_->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan",
            /*sensor_qos*/ 10,
            std::bind(&DeviceTopicMonitor::scanCallback, this, std::placeholders::_1)
        )
    );

    RCLCPP_INFO(logger_, "DeviceTopicMonitor initialized with topic subscriptions.");
}

void DeviceTopicMonitor::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    //5,000ms
    RCLCPP_INFO_THROTTLE(logger_, *node_->get_clock(), 5000, "[Odom] x=%.2f, y=%.2f", msg->pose.pose.position.x, msg->pose.pose.position.y);
}

#if _USED_
void DeviceTopicMonitor::bmsBatteryCallback(const lrbot2_msgs::msg::BmsStatus::SharedPtr msg)
{
    RCLCPP_INFO_THROTTLE(logger_, *node_->get_clock(), 5000, "[BMS] Voltage=%.2f, Current=%.2f", msg->voltage, msg->current);
}
#endif

void DeviceTopicMonitor::encoderCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    RCLCPP_INFO_THROTTLE(logger_, *node_->get_clock(), 5000, "[Encoder] joints=%ld", msg->name.size());
}

void DeviceTopicMonitor::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    RCLCPP_INFO_THROTTLE(logger_, *node_->get_clock(), 5000, "[IMU] Angular z=%.3f", msg->angular_velocity.z);
}

void DeviceTopicMonitor::scanFrontCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    RCLCPP_INFO_THROTTLE(logger_, *node_->get_clock(), 5000, "[Scan Front] size=%ld", msg->ranges.size());
}

void DeviceTopicMonitor::scanRearCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    RCLCPP_INFO_THROTTLE(logger_, *node_->get_clock(), 5000, "[Scan Rear] size=%ld", msg->ranges.size());
}

void DeviceTopicMonitor::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    RCLCPP_INFO_THROTTLE(logger_, *node_->get_clock(), 5000, "[Scan Merged] size=%ld", msg->ranges.size());
}
