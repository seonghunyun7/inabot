#ifndef PURE_PURSUIT_HPP
#define PURE_PURSUIT_HPP

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "inabot_msgs/msg/obstacle_status.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>
#include "geometry_msgs/msg/pose.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"
#include <tf2/LinearMath/Matrix3x3.h>

#include <vector>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <math.h>
#include <limits>

#include "planner/common/path_provider.hpp"

class pure_pursuit : public rclcpp::Node {
public:
  explicit pure_pursuit(const rclcpp::NodeOptions& options);

private:
  void initParameters();

  void amcl_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void path_callback(const nav_msgs::msg::Path::SharedPtr msg);
  void path_point_callback(const geometry_msgs::msg::Point::SharedPtr msg);

  void updateControl();
  int searchTargetIndex();
  geometry_msgs::msg::Twist purePursuitControl(int target_index);
  
  void publishCmd(const geometry_msgs::msg::Twist & cmd);
  void visualizePathAndRobot(int target_index);
  void publishGoalReached(bool reached);
  double quaternion_to_euler(const geometry_msgs::msg::Quaternion& quat);

  std::shared_ptr<PathProvider> path_provider_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr goal_reached_pub_;

  rclcpp::Subscription<inabot_msgs::msg::ObstacleStatus>::SharedPtr obstacle_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr path_point_sub_;
  // 장애물 감지 거리 구독자 선언
  //rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr obstacle_dist_sub_;

  geometry_msgs::msg::Pose current_pose_;
  geometry_msgs::msg::Twist current_velocity_;
  nav_msgs::msg::Path path_;
  
  // =======================
  // 장애물 정보 
  double obstacle_distance_ = 0.0;
  bool obstacle_stop_ = false;
  double min_safe_distance_ = 0.2;   // 최소 안전 거리 (m)
  double max_stop_distance_ = 1.0;   // 최대 감속 거리 (m)
  // =======================

  rclcpp::TimerBase::SharedPtr timer_;
  
  double lookahead_distance_;
  double max_linear_velocity_;
  double max_angular_velocity_;
  double control_rate_;
  double min_lookahead_distance_;
  double max_lookahead_distance_;
  double goal_threshold_;
  int current_index_;
};

#endif // PURE_PURSUIT_HPP
