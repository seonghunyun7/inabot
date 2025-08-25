#ifndef ODOMETRY_HPP_
#define ODOMETRY_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float32.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <chrono> 
#include <string>
#include <utility>
#include <cmath> // for M_PI
#include <memory>
#include <vector>

#include "kinematics/kinematics_factory.hpp"
#include "kinematics/base_kinematics.hpp"

#include "wheel/Wheel_manager_factory.hpp"
//#include "wheel/WheelManagerFactory.hpp"
#include "wheel/i_wheel_manager.hpp"

class OdometryNode : public rclcpp::Node
{
public:
    explicit OdometryNode(const rclcpp::NodeOptions& options);

private:
    void initParameters();
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void publishOdometry();

    float calculateMotorLoad(const std::vector<int>& pulses, const std::vector<double>& wheel_velocities);

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr motor_load_pub_;

    rclcpp::TimerBase::SharedPtr timer_;

    std::unique_ptr<kinematics::BaseKinematics> kinematics_;
    std::unique_ptr<IWheelManager> wheel_manager_;

    std::vector<double> current_wheel_velocities_;

    // 로봇 위치 상태 (간단화: x, y, theta)
    double x_ = 0.0;
    double y_ = 0.0;
    double theta_ = 0.0;
    std::vector<int> last_pulses_;  // 이전 pulse 저장
    float last_pulse_load_ = 0.0f;

    rclcpp::Time last_time_;

    //imu
    double imu_yaw_;
    bool imu_data_ready_ = false;
    bool use_combine_imu_;

    std::string drive_type_;
    double wheel_radius_;
    double wheel_base_;
    double wheel_separation_;
    int pulse_per_revolution_;

    WheelType wheel_type_;    // enum class WheelType { DIFFERENTIAL, STEERABLE };
};

#endif  // ODOMETRY_HPP_
