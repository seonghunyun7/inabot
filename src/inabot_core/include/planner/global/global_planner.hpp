#ifndef GLOBAL_PLANNER_ASTAR_HPP
#define GLOBAL_PLANNER_ASTAR_HPP

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "std_msgs/msg/float64.hpp"
#include <std_msgs/msg/bool.hpp>

#include "inabot_msgs/msg/obstacle_status.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> // tf2::getYaw
#include <tf2/utils.h>

#include <vector>

//#include "astar_planner.hpp"
#include "planner/global/astar_planner.hpp"

class GlobalPlanner : public rclcpp::Node
{
public:
  explicit GlobalPlanner(const rclcpp::NodeOptions& options);

private:
  void initParameters();
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void fmsPathCallback(const nav_msgs::msg::Path::SharedPtr msg);
  void tryPublishBestPath();
  void tryPublishPath();
  void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void obstacle_distance_callback(const std_msgs::msg::Float64::SharedPtr msg);
  bool worldToMap(const geometry_msgs::msg::Point& world_pt, int& mx, int& my);

  bool isPathValid(const nav_msgs::msg::Path& path);
  void planAndPublishAstarPath();
  std::vector<GridPoint> planAstarPath();

  bool isValid(const GridPoint& p);
  void saveFmsPathToFile();

  // 장애물 stop 토픽 publisher
  rclcpp::Publisher<inabot_msgs::msg::ObstacleStatus>::SharedPtr obstacle_pub_;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr fms_path_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr planned_path_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr obstacle_dist_sub_;

  nav_msgs::msg::OccupancyGrid::SharedPtr map_msg_;
  nav_msgs::msg::Path fms_path_msg_;
 
  bool map_received_;
  bool fms_path_received_;

  // 파라미터 변수들
  bool enable_a_start_;
  double min_safe_distance_;
  bool allow_unknown_area_;
  int path_point_sampling_interval_;
 
  //장애물 : 라이더 거리 기반
  double obstacle_distance_;
  bool obstacle_stop_ = false;
  double obstacle_stop_threshold_ = 0.5;
  double obstacle_resume_threshold_ = 1.0; // 1m

  // A* 알고리즘 클래스 인스턴스
  AstarPlanner astar_planner_;

  // 시작과 목표 위치 (geometry_msgs::msg::Pose)
  geometry_msgs::msg::Pose start_;
  geometry_msgs::msg::Pose goal_;

  nav_msgs::msg::Odometry current_odom_;
  bool odom_received_ = false;
};

#endif // GLOBAL_PLANNER_ASTAR_HPP
