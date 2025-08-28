#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <tf2_ros/create_timer_ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <cmath>
#include <string>
#include <vector>
#include <array>
#include <algorithm>

#define use_time_synchronizer

using namespace message_filters;
using SyncPolicy = sync_policies::ApproximateTime<sensor_msgs::msg::LaserScan, sensor_msgs::msg::LaserScan>;

struct LaserPoint
{
    float direction_;
    float distance_;
};

struct LaserPointLess
{
    bool operator()(const LaserPoint& a, const LaserPoint& b) const noexcept {
        return a.direction_ < b.direction_;
    }
};
 
class scanMerger : public rclcpp::Node
{
public:
    explicit scanMerger(const rclcpp::NodeOptions& options);

private:
    void initialize_params();
    void refresh_params();

#if !defined(use_time_synchronizer)
    void scan_callback1(const sensor_msgs::msg::LaserScan::SharedPtr _msg);
    void scan_callback2(const sensor_msgs::msg::LaserScan::SharedPtr _msg);
#else
    void synchronized_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr &laser1_msg,
                               const sensor_msgs::msg::LaserScan::ConstSharedPtr &laser2_msg);
#endif
    void publishOccupancyGrid(const std::vector<std::array<float,2>>& scan_data);
    void update_point_cloud_rgb();
    void removePointsWithinRadius(std::vector<float>& ranges, float radius, float angle_min, float angle_increment);

    float GET_R(float x, float y);
    float GET_THETA(float x, float y);
    float normalizeAngle(float angle);
    float interpolate(float angle_1, float angle_2, float magnitude_1, float magnitude_2, float current_angle);
    float selectMin(float magnitude_1, float magnitude_2);
    void geometry_quat_to_rpy(double* roll, double* pitch, double* yaw, geometry_msgs::msg::Quaternion geometry_quat);

    std::string topic1_, topic2_, integratedTopic_, integratedFrameId_;
    bool show1_, show2_;
    float laser1XOff_, laser1YOff_, laser1Yaw_;
    float laser2XOff_, laser2YOff_, laser2Yaw_;
    float robotFrontEnd_, robotRearEnd_, robotRightEnd_, robotLeftEnd_;
    float remove_radius_;
    bool use_time_synchronizer_;
    bool publish_occupancy_grid_;

#if defined(use_time_synchronizer)
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::LaserScan>> laser1_sub_, laser2_sub_;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
#else
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub1_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub2_;
#endif

    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_pub_;

    std::unique_ptr<tf2_ros::Buffer> tf2_;
    std::unique_ptr<tf2_ros::TransformListener> tf2_listener_;

#if !defined(use_time_synchronizer)
    sensor_msgs::msg::LaserScan::SharedPtr laser1_;
    sensor_msgs::msg::LaserScan::SharedPtr laser2_;
#else
    sensor_msgs::msg::LaserScan::ConstSharedPtr laser1_;
    sensor_msgs::msg::LaserScan::ConstSharedPtr laser2_;
#endif

    geometry_msgs::msg::TransformStamped trans1_;
    geometry_msgs::msg::TransformStamped trans2_;

    double tolerance_;
};

