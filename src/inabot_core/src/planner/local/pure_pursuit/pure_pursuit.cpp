#include "planner/local/pure_pursuit/pure_pursuit.hpp"
#include <cmath>
#include <algorithm>

#define __VISUAL_DEBUG__ false
#define __LOG__  false
#define __stop_obstacle__ false
#define __LOG_2_ false

#define SINGLE_POINT_MODE 0  // 1: 단일 포인트 처리, 0: 다중 포인트 처리

pure_pursuit::pure_pursuit(const rclcpp::NodeOptions& options)
  : Node("pure_pursuit", options), 
  current_index_(0)
{
  initParameters();

  path_provider_ = std::make_shared<PathProvider>(this->get_logger());

  #if __USED_AMCL__ // 속도 성분이 없다.. 음.
  pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/amcl_pose", rclcpp::QoS(10),
    std::bind(&pure_pursuit::amcl_pose_callback, this, std::placeholders::_1));
  #endif

  #if _SET_QOS_
  auto odom_qos = rclcpp::QoS(rclcpp::KeepLast(10))
                        .best_effort()
                        .durability_volatile();
  #endif

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "/odom", /*odom_qos*/ 10,
    std::bind(&pure_pursuit::odometry_callback, this, std::placeholders::_1));
  
  //경로를 전달
  //ros2 topic pub --once /plan nav_msgs/msg/Path "{header: {frame_id: 'map'}, poses: [{header: {stamp: now, frame_id: 'map'}, pose: {position: {x: 1.0, y: 1.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}]}"
  path_sub_ = create_subscription<nav_msgs::msg::Path>(
    "/planned_path", rclcpp::QoS(10),
    std::bind(&pure_pursuit::path_callback, this, std::placeholders::_1));

  // 장애물 정보
  obstacle_sub_ = create_subscription<inabot_msgs::msg::ObstacleStatus>(
    "/obstacle_status", 10,
    [this](const inabot_msgs::msg::ObstacleStatus::SharedPtr msg){
        if(msg) {
            obstacle_distance_ = msg->distance;
            obstacle_stop_ = msg->stop;
            #if __LOG__
            RCLCPP_INFO(this->get_logger(),
                        "[ObstacleStatus] distance: %.3f m, stop: %s",
                        obstacle_distance_,
                        obstacle_stop_ ? "true" : "false");
            #endif
        }
    });

#if __DEBUG__
  //사용자가 지정한 단일 포인트를 전달
  path_point_sub_ = create_subscription<geometry_msgs::msg::Point>(
    "/clicked_point", rclcpp::QoS(10),
    std::bind(&pure_pursuit::path_point_callback, this, std::placeholders::_1));
#endif

  //publisher
  cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(
    "/cmd_vel", rclcpp::QoS(10));

  #if  __VISUAL_DEBUG__
  marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
    "/visualization_marker", rclcpp::QoS(10));
  #endif

  goal_reached_pub_ = create_publisher<std_msgs::msg::Bool>(
    "/goal_reached", rclcpp::QoS(10));

  timer_ = create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(1000.0 / control_rate_)), //50 = control_rate_ -> 20ms
    std::bind(&pure_pursuit::updateControl, this));
}

void pure_pursuit::initParameters()
{
  // 파라미터 선언
  this->declare_parameter<double>("lookahead_distance", 1.0);
  this->declare_parameter<double>("max_linear_velocity", 0.5);
  this->declare_parameter<double>("max_angular_velocity", 1.0);
  this->declare_parameter<double>("control_rate", 50.0);
  this->declare_parameter<double>("min_lookahead_distance", 0.3);
  this->declare_parameter<double>("max_lookahead_distance", 2.0);
  this->declare_parameter<double>("goal_threshold", 0.1);
  this->declare_parameter<double>("obstacle_stop_threshold", 0.5);
    
  this->declare_parameter("min_safe_distance", 0.2);
  this->declare_parameter("max_stop_distance", 1.0);

  // 파라미터 읽기
  this->get_parameter("lookahead_distance", lookahead_distance_);
  this->get_parameter("max_linear_velocity", max_linear_velocity_);
  this->get_parameter("max_angular_velocity", max_angular_velocity_);
  this->get_parameter("control_rate", control_rate_);
  this->get_parameter("min_lookahead_distance", min_lookahead_distance_);
  this->get_parameter("max_lookahead_distance", max_lookahead_distance_);
  this->get_parameter("goal_threshold", goal_threshold_);
  this->get_parameter("min_safe_distance", min_safe_distance_);
  this->get_parameter("max_stop_distance", max_stop_distance_);

  // 읽은 값 로그 출력
  RCLCPP_INFO(this->get_logger(), "Lookahead Distance: %.2f", lookahead_distance_);
  RCLCPP_INFO(this->get_logger(), "Max Linear Velocity: %.2f m/s", max_linear_velocity_);
  RCLCPP_INFO(this->get_logger(), "Max Angular Velocity: %.2f rad/s", max_angular_velocity_);
  RCLCPP_INFO(this->get_logger(), "Control Rate: %.2f Hz", control_rate_);
  RCLCPP_INFO(this->get_logger(), "Min Lookahead Distance: %.2f m", min_lookahead_distance_);
  RCLCPP_INFO(this->get_logger(), "Max Lookahead Distance: %.2f m", max_lookahead_distance_);
  RCLCPP_INFO(this->get_logger(), "Goal Threshold: %.2f m", goal_threshold_);
  //RCLCPP_INFO(this->get_logger(), "Obstacle Stop Threshold: %.2f m", obstacle_stop_threshold_);
}

void pure_pursuit::amcl_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
  current_pose_ = msg->pose.pose;
}

void pure_pursuit::odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_pose_ = msg->pose.pose;  // Odometry로부터 위치 정보 받기
    current_velocity_ = msg->twist.twist;  // Odometry로부터 속도 정보 받기
}

void pure_pursuit::path_callback(const nav_msgs::msg::Path::SharedPtr msg) {
    if (msg->poses.empty()) {
      RCLCPP_WARN(this->get_logger(), "Received empty path.");
      return;
    }

#if 1 //__LOG__
    // 전체 경로 포인트 개수 출력
    RCLCPP_INFO(this->get_logger(), "Received path with %zu points", msg->poses.size());

    // 모든 포인트 좌표 출력
    for (size_t i = 0; i < msg->poses.size(); ++i) {
        const auto& pos = msg->poses[i].pose.position;
        RCLCPP_INFO(this->get_logger(), "Point %zu: (%.3f, %.3f, %.3f)", i, pos.x, pos.y, pos.z);
    }
#endif
    path_provider_->updatePath(*msg);
}  
  
void pure_pursuit::path_point_callback(const geometry_msgs::msg::Point::SharedPtr msg) {
  geometry_msgs::msg::PoseStamped point;
  point.header.stamp = now();
  point.header.frame_id = "map";
  point.pose.position = *msg;
  point.pose.orientation.w = 1.0;
  path_.poses.push_back(point);
}

double pure_pursuit::quaternion_to_euler(const geometry_msgs::msg::Quaternion& quat)
{
    tf2::Quaternion q;
    tf2::fromMsg(quat, q);

    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    return yaw; // Return yaw as theta
}

#if 0 //SINGLE_POINT_MODE
void pure_pursuit::updateControl() {
  // 경로가 비어있으면 제어 업데이트 건너뜀
  if (path_provider_->size() == 0) {
    //RCLCPP_INFO(this->get_logger(), "Path is empty, skipping control update.");
    return;
  }
 
  // target 포인트 탐색
  int target_index = searchTargetIndex(); // 단일 포인트 시나리오: 항상 0번 인덱스 반환
  if (target_index < 0 || target_index >= static_cast<int>(path_provider_->size())) {
    RCLCPP_WARN(this->get_logger(), "Invalid target_index %d (path size: %zu), skipping control update.",
                target_index, path_provider_->size());
    return;
  }

  #if __LOG_2_
  RCLCPP_INFO(this->get_logger(), "Current pose: (%.2f, %.2f)", current_pose_.position.x, current_pose_.position.y);
  auto target_pose = path_provider_->getPose(target_index);
  RCLCPP_INFO(this->get_logger(), "Target index: %d, Target point: (%.2f, %.2f)",
              target_index,
              target_pose.pose.position.x,
              target_pose.pose.position.y);
  #endif

  // 제어 명령 계산 및 발행
  auto cmd_vel = purePursuitControl(target_index); // <- index =  0
  #if __LOG_2_
  RCLCPP_INFO(this->get_logger(), "Publishing cmd_vel: linear.x=%.2f, angular.z=%.2f", cmd_vel.linear.x, cmd_vel.angular.z);
  #endif

  // 장애물 거리 기반 감속
  if (obstacle_stop_) {
      if (obstacle_distance_ < max_stop_distance_) {
          double decel_ratio = (obstacle_distance_ - min_safe_distance_) / (max_stop_distance_ - min_safe_distance_);
          decel_ratio = std::clamp(decel_ratio, 0.0, 1.0);

          double original_linear = cmd_vel.linear.x;
          cmd_vel.linear.x *= decel_ratio;

          RCLCPP_INFO(this->get_logger(),
              "[Obstacle] distance: %.2f m, decel_ratio: %.2f, linear.x: %.2f -> %.2f, angular.z: %.2f",
              obstacle_distance_, decel_ratio, original_linear, cmd_vel.linear.x, cmd_vel.angular.z);

          if (decel_ratio == 0.0) {
              cmd_vel.angular.z = 0.0; // 완전 정지 시 회전도 멈춤
              publishCmd(cmd_vel);
              RCLCPP_WARN(this->get_logger(), "Obstacle too close: %.2f m -> STOP", obstacle_distance_);
              return; // 장애물 정지 시 나머지 제어 루틴 건너뜀
          }
      }
  }
  //
  publishCmd(cmd_vel);

  #if  __VISUAL_DEBUG__
  visualizePathAndRobot(target_index);
  #endif

  // 목표 도달 여부 판정
  auto target_pose = path_provider_->getPose(target_index);
  double dx = target_pose.pose.position.x - current_pose_.position.x;
  double dy = target_pose.pose.position.y - current_pose_.position.y;
  double distance_to_goal = std::hypot(dx, dy);

  #if 1 //__LOG_2_
  RCLCPP_INFO(this->get_logger(), "Distance to goal: %.3f m, Threshold: %.3f m", distance_to_goal, goal_threshold_);
  #endif
  if (distance_to_goal <= goal_threshold_) {
    #if 1 // __LOG_2_
    RCLCPP_INFO(this->get_logger(), "Goal reached! Clearing path and resetting index.");
    #endif
    publishGoalReached(true);
    //더 이상 추적할 포인트가 없다면
    path_provider_->clearPath();
    current_index_ = 0;
  }
}
#endif

//단일 및 다중 경로 포인트도 모두 다 된다.
void pure_pursuit::updateControl() {
    // 경로가 비어있으면 제어 건너뜀
    if (path_provider_->size() == 0) {
        return;
    }

    // current_index_ 유효 범위 체크
    if (current_index_ >= static_cast<int>(path_provider_->size())) {
        RCLCPP_WARN(this->get_logger(), "Current index %d out of range (path size: %zu), clearing path.", 
                    current_index_, path_provider_->size());
        path_provider_->clearPath();
        current_index_ = 0;
        return;
    }

    // 현재 목표 포인트 가져오기
    auto target_pose = path_provider_->getPose(current_index_);

#if __LOG_2_
    RCLCPP_INFO(this->get_logger(), "Current pose: (%.2f, %.2f)", current_pose_.position.x, current_pose_.position.y);
    RCLCPP_INFO(this->get_logger(), "Target index: %d, Target point: (%.2f, %.2f)",
                current_index_, target_pose.pose.position.x, target_pose.pose.position.y);
#endif

    // 제어 명령 계산
    auto cmd_vel = purePursuitControl(current_index_);

    // 장애물 감속 처리
    if (obstacle_stop_ && obstacle_distance_ < max_stop_distance_) {
        double decel_ratio = (obstacle_distance_ - min_safe_distance_) / (max_stop_distance_ - min_safe_distance_);
        decel_ratio = std::clamp(decel_ratio, 0.0, 1.0);

        double original_linear = cmd_vel.linear.x;
        cmd_vel.linear.x *= decel_ratio;

        RCLCPP_INFO(this->get_logger(),
                    "[Obstacle] distance: %.2f m, decel_ratio: %.2f, linear.x: %.2f -> %.2f, angular.z: %.2f",
                    obstacle_distance_, decel_ratio, original_linear, cmd_vel.linear.x, cmd_vel.angular.z);

        if (decel_ratio == 0.0) {
            cmd_vel.angular.z = 0.0;
            publishCmd(cmd_vel);
            RCLCPP_WARN(this->get_logger(), "Obstacle too close: %.2f m -> STOP", obstacle_distance_);
            return;
        }
    }

    publishCmd(cmd_vel);

#if __VISUAL_DEBUG__
    visualizePathAndRobot(current_index_);
#endif

    // 마지막 포인트 도달 여부만 체크
    if (current_index_ == static_cast<int>(path_provider_->size()) - 1) {
        auto last_pose = path_provider_->getPose(current_index_);
        double dx = last_pose.pose.position.x - current_pose_.position.x;
        double dy = last_pose.pose.position.y - current_pose_.position.y;
        double distance_to_goal = std::hypot(dx, dy);
#if 1 //__LOG_2_
        RCLCPP_INFO(this->get_logger(), "Distance to final goal: %.3f m, Threshold: %.3f m", distance_to_goal, goal_threshold_);
#endif
        if (distance_to_goal <= goal_threshold_) {
            RCLCPP_INFO(this->get_logger(), "Final goal reached! Clearing path.");
            publishGoalReached(true);
            path_provider_->clearPath();
            current_index_ = 0;
        }
    } else {
        // 중간 포인트는 도달 시 인덱스 증가만
        double dx = target_pose.pose.position.x - current_pose_.position.x;
        double dy = target_pose.pose.position.y - current_pose_.position.y;
        double distance_to_point = std::hypot(dx, dy);

        if (distance_to_point <= goal_threshold_) {
            RCLCPP_INFO(this->get_logger(), "Reached intermediate point %d", current_index_);
            current_index_++;
        }
    }
}

int pure_pursuit::searchTargetIndex() {
#if __MULTI_PATH_
  double min_dist = std::numeric_limits<double>::max();
  int index = -1;

  for (size_t i = current_index_; i < path_provider_->size(); ++i) {
    auto pose = path_provider_->getPose(i);
    double dx = pose.pose.position.x - current_pose_.position.x;
    double dy = pose.pose.position.y - current_pose_.position.y;
    double dist = std::hypot(dx, dy);

    if (dist >= lookahead_distance_) {
      current_index_ = i;
      return i;
    }
    if (dist < min_dist) {
      min_dist = dist;
      index = i;
    }
  }
  return index;
#else
  if (path_provider_->size() == 0) return -1;
  return 0;
#endif
}
    
geometry_msgs::msg::Twist pure_pursuit::purePursuitControl(int target_index) {
  geometry_msgs::msg::Twist cmd;

  if (path_provider_->size() == 0) {
    return cmd;
  }

  // 단일 포인트 시나리오에서 첫 번째 점만 추적
  //목표 지점(path_.poses[target_index])과 현재 위치(current_pose_) 간의 x와 y 좌표 차이
  auto target_pose = path_provider_->getPose(target_index);
  double dx = target_pose.pose.position.x - current_pose_.position.x;
  double dy = target_pose.pose.position.y - current_pose_.position.y;
  double yaw = tf2::getYaw(current_pose_.orientation);

  // 로봇의 로컬 좌표계에서 목표 지점의 위치를 변환한 값.
  // 목표 지점까지의 거리와 방향을 계산
  double tx = std::cos(-yaw) * dx - std::sin(-yaw) * dy;
  double ty = std::sin(-yaw) * dx + std::cos(-yaw) * dy;

  // lookahead_distance를 속도 기반으로 동적 조정
  //속도에 맞춰 lookahead_distance를 조정:
  //로봇의 현재 속도에 따라 경로 추적을 위한 목표 지점까지의 거리를 동적으로 조정합니다. 
  //고속일 때는 긴 거리를, 저속일 때는 짧은 거리를 추적합니다.
  double linear_velocity = std::hypot(current_velocity_.linear.x, current_velocity_.linear.y);
  lookahead_distance_ = std::clamp(lookahead_distance_ * (linear_velocity / max_linear_velocity_),
                                   min_lookahead_distance_, max_lookahead_distance_);
  //목표 지점과의 거리 계산: tx <= 0.0이면 목표 지점이 뒤쪽에 있거나 이미 도달한 경우이므로 로봇을 멈추
  if (tx <= 0.0) {
    cmd.linear.x = 0.0;
    cmd.angular.z = 0.0;
    return cmd;
  }

  //목표 지점에 도달하지 않으면 각속도를 설정
  //curvature는 로봇이 목표 지점으로 향할 때 얼마나 빠르게 회전해야 하는지 계산하는 값
  //ty는 목표 지점의 y축 좌표 차이, lookahead_distance_는 목표 지점까지의 거리
  double curvature = 2.0 * ty / (lookahead_distance_ * lookahead_distance_);
  cmd.linear.x = max_linear_velocity_;
  cmd.angular.z = std::clamp(curvature * cmd.linear.x, -max_angular_velocity_, max_angular_velocity_);
  
  return cmd;
}

void pure_pursuit::publishCmd(const geometry_msgs::msg::Twist & cmd) {
  cmd_pub_->publish(cmd);
}

void pure_pursuit::visualizePathAndRobot(int target_index) {
  if (path_provider_->size() == 0) return;

  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = now();
  marker.ns = "target_point";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  
  auto pose = path_provider_->getPose(target_index);
  marker.pose = pose.pose;

  marker.scale.x = 0.2;
  marker.scale.y = 0.2;
  marker.scale.z = 0.2;
  marker.color.a = 1.0;
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker_pub_->publish(marker);
}

void pure_pursuit::publishGoalReached(bool reached) {
  std_msgs::msg::Bool msg;
  msg.data = reached;
  goal_reached_pub_->publish(msg);
}
