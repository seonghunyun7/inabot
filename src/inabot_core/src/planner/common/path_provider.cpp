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

/**
 * @brief FMS 등에서 받은 단일 경로 포인트를 받아 스플라인 보간 경로 생성
 * @param input_poses FMS에서 받은 전체 포즈 벡터
 * @param current_pose 차량 현재 위치
 * @param interp_spacing 보간 간격 (m)
 * @param min_points 최소 점 개수 (MPC용)
 * @return 보간된 nav_msgs::msg::Path
 */
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

/**
 * @brief FMS 등에서 받은 다수의 경로 포인트를 받아 스플라인 보간 경로 생성
 * @param input_poses FMS에서 받은 전체 포즈 벡터
 * @param current_pose 차량 현재 위치
 * @param interp_spacing 보간 간격 (m)
 * @param min_points 최소 점 개수 (MPC용)
 * @return 보간된 nav_msgs::msg::Path
 */
nav_msgs::msg::Path PathProvider::generateSplinePathFromMultiplePoints(
    const std::vector<geometry_msgs::msg::PoseStamped>& input_poses,
    const geometry_msgs::msg::Pose& current_pose, // 로봇 현재 위치 추가
    double interp_spacing,  // 보간 간격 (예: 0.4m)
    int min_points          // 최소 점 개수 (예: 4)
)
{
    nav_msgs::msg::Path path;

    if (input_poses.empty()) {
        RCLCPP_WARN(logger_, "[PathProvider] Input poses empty, returning empty path.");
        return path;
    }

    geometry_msgs::msg::Pose last_pose = current_pose; // 현재 로봇 위치 기준

    for (const auto& goal_pose : input_poses) {
        double dx = goal_pose.pose.position.x - last_pose.position.x;
        double dy = goal_pose.pose.position.y - last_pose.position.y;
        double dist = std::hypot(dx, dy);

        if (dist < 0.01) {
            path.poses.push_back(goal_pose);
            last_pose = goal_pose.pose;
            continue;
        }

        int interp_count = std::max(min_points - 1, static_cast<int>(std::floor(dist / interp_spacing)));

        tf2::Quaternion q_start, q_goal, q_interp;
        tf2::fromMsg(last_pose.orientation, q_start);
        tf2::fromMsg(goal_pose.pose.orientation, q_goal);

        for (int i = 0; i <= interp_count; ++i) {
            double t = static_cast<double>(i) / interp_count;

            geometry_msgs::msg::PoseStamped pose_stamped;
            pose_stamped.header = goal_pose.header;

            // 위치 선형 보간
            pose_stamped.pose.position.x = (1 - t) * last_pose.position.x + t * goal_pose.pose.position.x;
            pose_stamped.pose.position.y = (1 - t) * last_pose.position.y + t * goal_pose.pose.position.y;
            pose_stamped.pose.position.z = 0.0;

            // 방향 구면 선형 보간 (Slerp)
            q_interp = q_start.slerp(q_goal, t);
            pose_stamped.pose.orientation = tf2::toMsg(q_interp);

            path.poses.push_back(pose_stamped);

            RCLCPP_INFO(logger_, "[PathProvider] Interp Point: x=%.3f, y=%.3f, z=%.3f",
                        pose_stamped.pose.position.x,
                        pose_stamped.pose.position.y,
                        pose_stamped.pose.position.z);
        }

        last_pose = goal_pose.pose;
    }

    if (!input_poses.empty()) {
        path.header = input_poses.back().header;
    }

    RCLCPP_INFO(logger_, "[PathProvider] Generated spline path with %zu points.", path.poses.size());
    return path;
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
