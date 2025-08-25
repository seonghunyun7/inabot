// src/planner/common/path_provider.cpp
#include <iostream>  // std::cout
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "planner/common/path_provider.hpp"

#define __LOG__ 0

PathProvider::PathProvider(rclcpp::Logger logger)
  : logger_(logger) {}

void PathProvider::updatePath(const nav_msgs::msg::Path & new_path) {
  std::lock_guard<std::mutex> lock(mutex_);
  path_points_ = new_path.poses;

#if __LOG__
  RCLCPP_INFO(logger_, "[PathProvider] updatePath called, new path size: %zu", path_points_.size());
  for (size_t i = 0; i < path_points_.size(); ++i) {
    const auto& p = path_points_[i].pose.position;
    RCLCPP_INFO(logger_, "  Point %zu: (%.3f, %.3f, %.3f)", i, p.x, p.y, p.z);
  }
#endif
}
 
void PathProvider::clearPath() {
  std::lock_guard<std::mutex> lock(mutex_);
  path_points_.clear();
#if __LOG__
  RCLCPP_INFO(logger_, "[PathProvider] clearPath called, path cleared.");
#endif
}

size_t PathProvider::size() const {
  std::lock_guard<std::mutex> lock(mutex_);
  size_t sz = path_points_.size();
#if __LOG__
  RCLCPP_INFO(logger_, "[PathProvider] size() called, current path size: %zu", sz);
#endif
  return sz;
}

geometry_msgs::msg::PoseStamped PathProvider::getPose(size_t index) const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (index >= path_points_.size()) {
    std::cout << "[PathProvider] getPose ERROR: index " << index << " out of range (size: " << path_points_.size() << ")" << std::endl;
    throw std::out_of_range("PathProvider: index out of range");
  }
  const auto& p = path_points_[index].pose.position;
#if __LOG__
  RCLCPP_INFO(logger_, "[PathProvider] getPose called, index: %zu, Point: (%.3f, %.3f, %.3f)", index, p.x, p.y, p.z);
#endif
  return path_points_[index];
}

const std::vector<geometry_msgs::msg::PoseStamped> & PathProvider::getPath() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return path_points_;
}

// 단일 목표 지점에 대해 현재 위치 기반 보간 경로 생성
// 단일 목표 지점에 대해 현재 위치 기반 보간 경로 생성
nav_msgs::msg::Path PathProvider::generateInterpolatedPathForSinglePoint(
    const geometry_msgs::msg::PoseStamped& goal_pose,
    const geometry_msgs::msg::Pose& current_pose,
    double interp_spacing,   // 보간 간격 (예: 0.4m)
    int min_points           // 최소 점 개수 (예: 4)
)
{
  nav_msgs::msg::Path path;
  path.header = goal_pose.header;

  double dx = goal_pose.pose.position.x - current_pose.position.x;
  double dy = goal_pose.pose.position.y - current_pose.position.y;
  double dist = std::hypot(dx, dy);

#if __LOG__
  RCLCPP_INFO(logger_, "[PathProvider] generateInterpolatedPathForSinglePoint: Dist = %.3f", dist);
#endif

  if (dist < 0.01) {
#if __LOG__
    RCLCPP_INFO(logger_, "[PathProvider] Distance < 1cm, returning single goal point.");
#endif
    path.poses.push_back(goal_pose);
    return path;
  }

  int interp_count = std::max(min_points - 1, static_cast<int>(std::floor(dist / interp_spacing)));

  tf2::Quaternion q_start, q_goal, q_interp;
  tf2::fromMsg(current_pose.orientation, q_start);
  tf2::fromMsg(goal_pose.pose.orientation, q_goal);

  for (int i = 0; i <= interp_count; ++i) {
    double t = static_cast<double>(i) / interp_count;

    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header = goal_pose.header;

    // 위치 선형 보간
    pose_stamped.pose.position.x = (1 - t) * current_pose.position.x + t * goal_pose.pose.position.x;
    pose_stamped.pose.position.y = (1 - t) * current_pose.position.y + t * goal_pose.pose.position.y;
    pose_stamped.pose.position.z = 0.0;

    // 방향 구면 선형 보간 (Slerp)
    q_interp = q_start.slerp(q_goal, t);
    pose_stamped.pose.orientation = tf2::toMsg(q_interp);

    path.poses.push_back(pose_stamped);
  }

  return path;
}

// 여러 점으로 구성된 경로에 대해 스플라인 보간 경로 생성
nav_msgs::msg::Path PathProvider::generateSplinePathFromMultiplePoints(
    const std::vector<geometry_msgs::msg::PoseStamped>& input_poses)
{
  nav_msgs::msg::Path interpolated_path;
  if (input_poses.empty()) return interpolated_path;

  interpolated_path.header = input_poses.front().header;

  size_t n = input_poses.size();
  if (n < 2) {
    if (n == 1) {
      interpolated_path.poses.push_back(input_poses[0]);
    }
    return interpolated_path;
  }

  int interp_points_between = std::max(1, static_cast<int>(std::ceil(3.0 / (n - 1))));

  for (size_t i = 0; i < n - 1; ++i) {
    const auto& p1 = input_poses[i].pose;
    const auto& p2 = input_poses[i + 1].pose;

    // 구간 시작점은 처음 한 번만 추가 (첫 구간일 때)
    if (i == 0) {
      interpolated_path.poses.push_back(input_poses[0]);
    }

    for (int j = 1; j <= interp_points_between; ++j) {
      float t = static_cast<float>(j) / (interp_points_between + 1);

      geometry_msgs::msg::PoseStamped interp_pose;
      interp_pose.header = input_poses[i].header;

      // 위치 선형 보간
      interp_pose.pose.position.x = (1 - t) * p1.position.x + t * p2.position.x;
      interp_pose.pose.position.y = (1 - t) * p1.position.y + t * p2.position.y;
      interp_pose.pose.position.z = 0.0;

      // orientation slerp 보간
      tf2::Quaternion q1, q2, q_interp;
      tf2::fromMsg(p1.orientation, q1);
      tf2::fromMsg(p2.orientation, q2);
      q_interp = q1.slerp(q2, t);
      interp_pose.pose.orientation = tf2::toMsg(q_interp);

      interpolated_path.poses.push_back(interp_pose);
    }
  }

  // 마지막 포인트 추가
  interpolated_path.poses.push_back(input_poses.back());

  return interpolated_path;
}

// 현재 위치와 목표 위치 간 도착 여부 판단
bool PathProvider::isWithinGoalThreshold(
    const geometry_msgs::msg::Pose& goal_pose,
    const geometry_msgs::msg::Pose& current_pose,
    double threshold) const
{
  double dx = goal_pose.position.x - current_pose.position.x;
  double dy = goal_pose.position.y - current_pose.position.y;
  double dist = std::hypot(dx, dy);

#if __LOG__
  RCLCPP_INFO(logger_, "[PathProvider] isWithinGoalThreshold: dist = %.3f, threshold = %.3f", dist, threshold);
#endif

  return dist < threshold;
}

// 경로 정보 로그 출력
void PathProvider::logGlobalPlanInfo(const nav_msgs::msg::Path& path) const
{
  if (path.poses.empty()) return;

  const auto& first_pose = path.poses.front().pose.position;
  const auto& last_pose = path.poses.back().pose.position;

  RCLCPP_INFO(logger_, "[PathProvider] Global plan: First point (%.3f, %.3f), Last point (%.3f, %.3f)",
              first_pose.x, first_pose.y, last_pose.x, last_pose.y);
}