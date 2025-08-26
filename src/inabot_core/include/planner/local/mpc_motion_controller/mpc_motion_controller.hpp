// mpc_motion_controller.hpp
#ifndef MPC_MOTION_CONTROLLER_HPP_
#define MPC_MOTION_CONTROLLER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/bool.hpp"

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include <geometry_msgs/msg/twist_stamped.hpp>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> // tf2::getYaw
#include <tf2/utils.h>
#include "geometry_msgs/msg/pose.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"
#include <tf2/LinearMath/Matrix3x3.h>

#include <vector>
#include <numeric>
#include <math.h>
#include <algorithm>  // std::clamp
#include <cmath>      // std::hypot
#include <Eigen/Core>
#include <Eigen/QR>
#include <iostream>
#include <Eigen/Sparse>
#include <Eigen/Dense>
#include <cstring>  // memset 
#include <memory>
#include <string>

#include <deque>
#include <mutex>

#include "inabot_msgs/msg/obstacle_status.hpp"

#include "planner/common/path_provider.hpp"
#include "planner/local/mpc_motion_controller/mpc_core.hpp"

class MpcMotionController : public rclcpp::Node
{
public:
    explicit MpcMotionController(const rclcpp::NodeOptions& options);
    ~MpcMotionController();

private:
    // 콜백
    void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);
    void timerCallback();

    void initParameters();
    bool isOdomValid() const;
    geometry_msgs::msg::TwistStamped computeVelocityCommands();

    //path of the fitging
    double polyeval(Eigen::VectorXd coeffs, double x);
    Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order);
    double impThetaError(double theta, const Eigen::VectorXd& coeffs,
                                    int sample_size, int sample_ratio);
    //goal target of the reach..
    void publishGoalReached(bool reached);

    // 구독자 & 퍼블리셔
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<inabot_msgs::msg::ObstacleStatus>::SharedPtr obstacle_sub_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr goal_reached_pub_;
    
    rclcpp::TimerBase::SharedPtr timer_;

    std::shared_ptr<PathProvider> path_provider_;

    // 내부 상태 저장
    geometry_msgs::msg::Pose current_pose_;
    geometry_msgs::msg::Twist current_velocity_;
    nav_msgs::msg::Path current_path_;
    geometry_msgs::msg::Twist control_cmd_;

    nav_msgs::msg::Path global_plan_;
    nav_msgs::msg::Odometry current_odom_;

    // 파라미터
    double max_angular_velocity_;
    double max_linear_velocity_;
    int mpc_horizon_;
    double mpc_dt_;
    double mpc_ref_velocity_;
    double goal_threshold_;

    // 상태 및 입력 제한 (필요에 따라 변경 가능)
    //solve mpc
    double x_min_ = -1e10, x_max_ = 1e10;
    double v_min_ = 0.0, v_max_ = 1.5;
    double a_min_ = -1.0, a_max_ = 1.0;
    double w_min_ = -1.0, w_max_ = 1.0;

        //solve mpc
    MPCCore _mpc;
    std::map<string, double> _mpc_params;
    
    double mpc_steps_, ref_cte_, ref_etheta_, ref_vel_, w_cte_, w_etheta_, w_vel_,
        w_angvel_, w_accel_, w_angvel_d_, w_accel_d_, max_angvel_, max_throttle_,
        bound_value_;

    double dt_, w_, throttle_, speed_, max_speed_;
    double pathLength_, goalRadius_, waypointsDist_;
    int controller_freq_, downSampling_, thread_numbers_;
    bool goal_received_, goal_reached_, path_computed_, pub_twist_flag_,
        debug_info_, delay_mode_;

    tf2::Duration transform_tolerance_;
    double control_duration_;
    bool use_cost_regulated_linear_velocity_scaling_;
    double inflation_cost_scaling_factor_;

    // 예: 클래스 멤버 변수 중
    // double obstacle_distance_;
    // bool obstacle_stop_ = false;
    // double obstacle_stop_threshold_ = 0.5;
    // double obstacle_resume_threshold_ = 0.7;

    double interp_spacing_;
    int min_points_;

    double obstacle_distance_ = std::numeric_limits<double>::infinity(); // 장애물 거리
    bool obstacle_stop_ = false;  // 장애물 감지 여부

    // 감속/정지 관련 파라미터
    double min_safe_distance_ = 0.2;   // 최소 안전 거리(m)
    double max_stop_distance_ = 1.0;   // 감속 시작 거리(m)

    std::deque<geometry_msgs::msg::PoseStamped> interpolated_path_queue_;
    
    std::vector<nav_msgs::msg::Path> interpolated_segments_;

    std::mutex path_mutex_;
};

#endif  // MPC_MOTION_CONTROLLER_HPP_
