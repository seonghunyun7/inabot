#include "odometry.hpp"
#include "kinematics/differential_drive.hpp"
#include "kinematics/kinematics_factory.hpp"
#include <tf2/LinearMath/Quaternion.h>

#include <utils/logger.h>

using namespace Inabot;

OdometryNode::OdometryNode(const rclcpp::NodeOptions& options)
: Node("odometry_node", options)
{
    initParameters();

    kinematics_ = kinematics::KinematicsFactory::create(drive_type_);
    if (!kinematics_) {
        RCLCPP_ERROR(this->get_logger(), "Unknown drive type: %s", drive_type_.c_str());
        // 노드 종료 혹은 예외처리 필요
    }

    kinematics_->setWheelRadius(wheel_radius_);
    kinematics_->setWheelBase(wheel_base_);
    kinematics_->setWheelSeparation(wheel_separation_);

    //WheelManager 생성
    if (drive_type_ == "differential") {
        wheel_type_ = WheelType::DIFFERENTIAL;
    } else if (drive_type_ == "traction_steering") {
        wheel_type_ = WheelType::STEERABLE;
    } else {
        RCLCPP_ERROR(this->get_logger(), "Unknown drive_type parameter");
        throw std::runtime_error("Invalid drive_type");
    }
    
    wheel_manager_ = WheelManagerFactory::createWheelManager(wheel_type_);

    // QoS settings for encoder
#if __SET_QoS_SET
    auto encoder_qos = rclcpp::QoS(rclcpp::SensorDataQoS());
    encoder_qos.keep_last(10);
#endif

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", 10 /*encoder_qos*/,
        std::bind(&OdometryNode::jointStateCallback, this, std::placeholders::_1));

#if __SET_QoS_SET
    rclcpp::QoS qos = rclcpp::QoS(rclcpp::SensorDataQoS());
    qos.keep_last(10);
#endif
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "imu", 10,
        std::bind(&OdometryNode::imuCallback, this, std::placeholders::_1));

#if __SET_QoS_SET
    // QoS settings for odometry publisher
    auto odom_qos = rclcpp::QoS(rclcpp::KeepLast(10))
                    .best_effort()
                    .durability_volatile();
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", odom_qos);
#else
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
#endif
    
    // motor load 퍼블리셔 추가 (부하 추정값 토픽)
    motor_load_pub_ = this->create_publisher<std_msgs::msg::Float32>("motor_load", 10);

    //10ms for cycle
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&OdometryNode::publishOdometry, this));

    last_time_ = this->now();
}

void OdometryNode::initParameters() 
{
    this->declare_parameter<std::string>("drive_type", "differential");
    this->declare_parameter<double>("wheel_radius", 0.05);
    this->declare_parameter<double>("wheel_base", 0.3);
    this->declare_parameter<double>("wheel_separation", 0.3);
    this->declare_parameter<int>("pulse_per_revolution", 4096);
    this->declare_parameter<bool>("use_combine_imu", false);

    this->get_parameter("drive_type", drive_type_);
    this->get_parameter("wheel_radius", wheel_radius_);
    this->get_parameter("wheel_base", wheel_base_);
    this->get_parameter("wheel_separation", wheel_separation_);
    this->get_parameter("pulse_per_revolution", pulse_per_revolution_);
    this->get_parameter("use_combine_imu", use_combine_imu_);

    RCLCPP_INFO(this->get_logger(), "drive_type: %s", drive_type_.c_str());
    RCLCPP_INFO(this->get_logger(), "wheel_radius: %.3f", wheel_radius_);
    RCLCPP_INFO(this->get_logger(), "wheel_base: %.3f", wheel_base_);
    RCLCPP_INFO(this->get_logger(), "wheel_separation: %.3f", wheel_separation_);
    RCLCPP_INFO(this->get_logger(), "pulse_per_revolution: %d", pulse_per_revolution_);
    RCLCPP_INFO(this->get_logger(), "Use IMU: %s", use_combine_imu_ ? "true" : "false");
}

void OdometryNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    if (std::isnan(msg->orientation.x) || std::isnan(msg->orientation.y) ||
        std::isnan(msg->orientation.z) || std::isnan(msg->orientation.w))
    {
        RCLCPP_INFO(this->get_logger(), "Received NaN in IMU orientation!");
        return;
    }

    #if __LOG__
    RCLCPP_INFO(this->get_logger(), "IMU Orientation (x, y, z, w): %.3f, %.3f, %.3f, %.3f", 
                msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
    #endif

    double imu_yaw_raw = atan2f(
        msg->orientation.x * msg->orientation.y + msg->orientation.w * msg->orientation.z,
        0.5f - msg->orientation.y * msg->orientation.y - msg->orientation.z * msg->orientation.z);

    imu_yaw_ = imu_yaw_raw;
    imu_data_ready_ = true; // 정상 데이터 수신

    #if __LOG__
    RCLCPP_INFO(this->get_logger(), "Final IMU Yaw: %.3f", imu_yaw_);
    #endif
}

void OdometryNode::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    std::vector<int> pulses;
    std::vector<double> velocities;

    // 예: msg->name 은 joint 이름 배열
    // pulses: raw encoder counts (position)
    // velocities: 속도 (m/s)

    for (size_t i = 0; i < msg->name.size(); ++i) {
        if (i >= msg->position.size() || i >= msg->velocity.size())
            continue;
        pulses.push_back(static_cast<int>(msg->position[i]));
        velocities.push_back(msg->velocity[i]);
    }

    if (wheel_manager_) {
        wheel_manager_->updateWheelData(pulses, velocities); //raw
        current_wheel_velocities_ = wheel_manager_->getWheelVelocities(); 
    }
}

void OdometryNode::publishOdometry()
{
    auto current_time = this->now();
    double dt = (current_time - last_time_).seconds();
    last_time_ = current_time;

    if (!wheel_manager_) return;

    // 현재 pulses 가져오기
    //STEERABLE : [front_traction, front_steering, rear_traction, rear_steering]
    //[1240, 512, 1320, 500]
    std::vector<int> current_pulses = wheel_manager_->getWheelPulses();
    if (last_pulses_.empty()) {
        last_pulses_ = current_pulses;
        return;
    }

    // delta pulse 계산
    std::vector<int> delta_pulses;
    for (size_t i = 0; i < current_pulses.size(); ++i) {
        delta_pulses.push_back(current_pulses[i] - last_pulses_[i]);
    }

    last_pulses_ = current_pulses;  // 업데이트

    // pulse → 거리 변환 (Steerable의 경우 traction만 사용)
    double front_dist = kinematics_->pulseToDistance(delta_pulses[0], pulse_per_revolution_);  // front traction
    double rear_dist = kinematics_->pulseToDistance(delta_pulses[2], pulse_per_revolution_);   // rear traction

    // 평균 이동 거리
    double dx = (front_dist + rear_dist) / 2.0;

    // 방향 각도 (steering angle)
    double front_angle = current_wheel_velocities_.size() > 0 ? current_wheel_velocities_[0] : 0.0;
    double rear_angle  = current_wheel_velocities_.size() > 2 ? current_wheel_velocities_[2] : 0.0;
    double heading = (front_angle + rear_angle) / 2.0;

    // 위치 갱신
    x_ += dx * cos(theta_ + heading);
    y_ += dx * sin(theta_ + heading);
    theta_ += 0.0;  // 회전은 실제로 계산할 steer geometry가 필요함 (여기선 생략)

    // 메시지 생성 및 발행은 그대로 유지
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = current_time;
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";

    odom_msg.pose.pose.position.x = x_;
    odom_msg.pose.pose.position.y = y_;
    odom_msg.pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, theta_);
    odom_msg.pose.pose.orientation.x = q.x();
    odom_msg.pose.pose.orientation.y = q.y();
    odom_msg.pose.pose.orientation.z = q.z();
    odom_msg.pose.pose.orientation.w = q.w();

    // velocity는 유지
    double linear_x = 0.0, linear_y = 0.0, angular_z = 0.0;
    kinematics_->wheelVelocitiesToTwist(current_wheel_velocities_, linear_x, linear_y, angular_z);
    odom_msg.twist.twist.linear.x = linear_x;
    odom_msg.twist.twist.linear.y = linear_y;
    odom_msg.twist.twist.angular.z = angular_z;

    odom_pub_->publish(odom_msg);

    // TF 발행
    geometry_msgs::msg::TransformStamped odom_tf;
    odom_tf.header.stamp = current_time;
    odom_tf.header.frame_id = "odom";
    odom_tf.child_frame_id = "base_link";

    odom_tf.transform.translation.x = x_;
    odom_tf.transform.translation.y = y_;
    odom_tf.transform.translation.z = 0.0;

    odom_tf.transform.rotation.x = q.x();
    odom_tf.transform.rotation.y = q.y();
    odom_tf.transform.rotation.z = q.z();
    odom_tf.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(odom_tf);
}