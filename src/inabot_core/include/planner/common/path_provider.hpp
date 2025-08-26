#ifndef PATH_PROVIDER_HPP_
#define PATH_PROVIDER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <mutex>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

class PathProvider {
public:
  PathProvider(rclcpp::Logger logger);

  // 경로 갱신
  void updatePath(const nav_msgs::msg::Path & new_path);

  // 현재 경로 비우기
  void clearPath();

  // 경로 크기 조회
  size_t size() const;

  // 인덱스에 따른 경로 포인트 반환 (복사본)
  geometry_msgs::msg::PoseStamped getPose(size_t index) const;

  // 내부 경로 참조 반환 (주의해서 사용)
  const std::vector<geometry_msgs::msg::PoseStamped> & getPath() const;

  // 단일 목표 지점에 대해 현재 위치 기반 보간 경로 생성
  nav_msgs::msg::Path generateInterpolatedPathForSinglePoint(
      const geometry_msgs::msg::PoseStamped& goal_pose,
      const geometry_msgs::msg::Pose& current_pose,
      double interp_spacing = 0.4,  // 기본 40cm 간격
      int min_points = 4             // 기본 최소 4개 점
  );

  /**
   * @brief FMS 등에서 받은 다수의 경로 포인트를 받아 스플라인 보간 경로 생성
   * @param input_poses FMS에서 받은 전체 포즈 벡터
   * @param current_pose 차량 현재 위치
   * @param interp_spacing 보간 간격 (m)
   * @param min_points 최소 점 개수 (MPC용)
   * @return 보간된 nav_msgs::msg::Path
   */
    nav_msgs::msg::Path generateSplinePathFromMultiplePoints(
        const std::vector<geometry_msgs::msg::PoseStamped>& input_poses,
        const geometry_msgs::msg::Pose& current_pose,
        double interp_spacing = 0.4,
        int min_points = 4
    );

  // 현재 위치와 목표 위치 간 도착 여부 판단
  bool isWithinGoalThreshold(
      const geometry_msgs::msg::Pose& goal_pose,
      const geometry_msgs::msg::Pose& current_pose,
      double threshold = 0.1) const;

  // 경로 정보 로그 출력
  void logGlobalPlanInfo(const nav_msgs::msg::Path& path) const;

private:
  rclcpp::Logger logger_;
  mutable std::mutex mutex_;
  std::vector<geometry_msgs::msg::PoseStamped> path_points_;
};

#endif // PATH_PROVIDER_HPP_

