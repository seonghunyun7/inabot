//#include "global_planner.hpp"
#include "planner/global/global_planner.hpp"
#include <fstream>
#include <iomanip>
#include <sstream>
#include <ctime>

GlobalPlanner::GlobalPlanner(const rclcpp::NodeOptions& options)
  : Node("global_planner_node", options), map_received_(false), fms_path_received_(false)
{
  initParameters();

  auto odom_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile();
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "/odom", odom_qos,
    std::bind(&GlobalPlanner::odometryCallback, this, std::placeholders::_1));

  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map", 10, std::bind(&GlobalPlanner::mapCallback, this, std::placeholders::_1));

  fms_path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    "/fms_path", 10, std::bind(&GlobalPlanner::fmsPathCallback, this, std::placeholders::_1));

  obstacle_dist_sub_ = create_subscription<std_msgs::msg::Float64>(
    "/obstacle_forward_distance", rclcpp::QoS(10),
    std::bind(&GlobalPlanner::obstacle_distance_callback, this, std::placeholders::_1));
 
  obstacle_pub_ = this->create_publisher<inabot_msgs::msg::ObstacleStatus>(
    "obstacle_status", 10);

  planned_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/planned_path", 10);
}

void GlobalPlanner::initParameters()
{
  this->declare_parameter<double>("min_safe_distance", 0.3);
  this->declare_parameter<bool>("allow_unknown_area", false);
  this->declare_parameter<int>("path_point_sampling_interval", 5);
  this->declare_parameter<double>("obstacle_stop_threshold", 0.5);
  this->declare_parameter<double>("obstacle_resume_threshold", 1.0);
  this->declare_parameter<bool>("enable_a_start", false);;

  this->get_parameter("min_safe_distance", min_safe_distance_);
  this->get_parameter("allow_unknown_area", allow_unknown_area_);
  this->get_parameter("path_point_sampling_interval", path_point_sampling_interval_);
  this->get_parameter("obstacle_stop_threshold", obstacle_stop_threshold_);
  this->get_parameter("obstacle_resume_threshold", obstacle_resume_threshold_);
  this->get_parameter("use_combine_imu", enable_a_start_ );

  // A* 플래너에도 파라미터 설정 전달
  astar_planner_.setMinSafeDistance(min_safe_distance_);
  astar_planner_.setAllowUnknownArea(allow_unknown_area_);

  RCLCPP_INFO(this->get_logger(), "GlobalPlanner node started");
  RCLCPP_INFO(this->get_logger(), "min_safe_distance: %f", min_safe_distance_);
  RCLCPP_INFO(this->get_logger(), "allow_unknown_area: %s", allow_unknown_area_ ? "true" : "false");
  RCLCPP_INFO(this->get_logger(), "path_point_sampling_interval: %d", path_point_sampling_interval_);
  RCLCPP_INFO(this->get_logger(), "Use enable_a_start: %s", enable_a_start_ ? "true" : "false");
}

void GlobalPlanner::obstacle_distance_callback(const std_msgs::msg::Float64::SharedPtr msg)
{
  if (!msg) {
    RCLCPP_WARN(this->get_logger(), "Received null obstacle distance message!");
    return;
  }

  obstacle_distance_ = msg->data;

  //히스테리시스(Hysteresis) 개념 적용:
  //멈출 때는 ≤ stop_threshold (예: 0.6m)
  //재개할 때는 > resume_threshold (예: 1.m)

  if (obstacle_distance_ <= obstacle_stop_threshold_ && !obstacle_stop_) {
    obstacle_stop_ = true;
    RCLCPP_WARN(this->get_logger(), "Obstacle detected: %.2f m -> STOP", obstacle_distance_);
  } else if (obstacle_distance_ > obstacle_resume_threshold_ && obstacle_stop_) {
    obstacle_stop_ = false;
    RCLCPP_INFO(this->get_logger(), "Obstacle cleared: %.2f m -> RESUME", obstacle_distance_);
  }

  inabot_msgs::msg::ObstacleStatus obs_msg;
  obs_msg.distance = obstacle_distance_;
  obs_msg.stop = obstacle_stop_;
  obstacle_pub_->publish(obs_msg);
}

void GlobalPlanner::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  if (!msg) {
    RCLCPP_WARN(this->get_logger(), "Received null odometry message");
    return;
  }

  //A star 알고리즘 일단 회피 경로 일단 조건으로 일단 라이더 장애물 인지로 간단히 가감속 중지로
  if (!enable_a_start_)
  {
    RCLCPP_WARN(this->get_logger(), "disable a start planner");
    return;
  }

  map_msg_ = msg;

  map_received_ = true;

  // A* 플래너에 맵 정보 전달
  astar_planner_.setMap(map_msg_->data, map_msg_->info.width, map_msg_->info.height, map_msg_->info.resolution);

  tryPublishBestPath();
}

void GlobalPlanner::odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  if (!msg) {
    RCLCPP_WARN(this->get_logger(), "Received null odometry message");
    return;
  }

  current_odom_ = *msg;
  odom_received_ = true;  // 첫 메시지 수신 시 true

  const auto& pos = msg->pose.pose.position;
  const auto& ori = msg->pose.pose.orientation;
  const auto& vel = msg->twist.twist.linear;

  double yaw = tf2::getYaw(ori);
#if __LOG__
  RCLCPP_INFO(this->get_logger(), "[Odometry] Pos: (%.2f, %.2f), Yaw: %.2f rad, Linear Vel: %.2f m/s",
                  pos.x, pos.y, yaw, vel.x);
#endif
}

void GlobalPlanner::fmsPathCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
  if (!msg) {
    RCLCPP_WARN(this->get_logger(), "Received null FMS path message");
    return;
  }

  fms_path_msg_ = *msg;
  fms_path_received_ = true;

  RCLCPP_INFO(this->get_logger(), "FMS Path received: %zu points", fms_path_msg_.poses.size());

  if (fms_path_msg_.poses.empty()) {
    RCLCPP_WARN(this->get_logger(), "Warning: Received empty FMS path!");
  } else {
    #if 0
    // 시작과 끝 좌표 출력
    auto& start_pose = fms_path_msg_.poses.front().pose.position;
    auto& goal_pose = fms_path_msg_.poses.back().pose.position;
    RCLCPP_INFO(this->get_logger(), "FMS Path start: (%.3f, %.3f), goal: (%.3f, %.3f)", 
                start_pose.x, start_pose.y, goal_pose.x, goal_pose.y);
    #endif
    // 모든 포인트 출력
    for (size_t i = 0; i < fms_path_msg_.poses.size(); ++i) {
      auto& pos = fms_path_msg_.poses[i].pose.position;
      RCLCPP_INFO(this->get_logger(), "Point %zu: (%.3f, %.3f, %.3f)", i, pos.x, pos.y, pos.z);
    }
  }

  // A* 알고리즘 조건에 따라 경로 발행
  if (!enable_a_start_) {
    tryPublishPath();
  } else {
    tryPublishBestPath();
  }
#if _path_save
  saveFmsPathToFile();
#endif
}

void GlobalPlanner::tryPublishBestPath()
{
  if (!map_received_ || !fms_path_received_) {
    RCLCPP_WARN(this->get_logger(), "Waiting for map and FMS path to be received");
    return;
  }

  if (fms_path_msg_.poses.empty()) {
    RCLCPP_WARN(this->get_logger(), "Received path is empty");
    return;
  }

  if (!isPathValid(fms_path_msg_)) {
    RCLCPP_INFO(this->get_logger(), "FMS path invalid, starting A* path planning");
    planAndPublishAstarPath();
  } else {
    RCLCPP_INFO(this->get_logger(), "FMS path valid, publishing it directly");
    planned_path_pub_->publish(fms_path_msg_);
  }
}

void GlobalPlanner::tryPublishPath()
{
  if (!fms_path_received_) {
    RCLCPP_WARN(this->get_logger(), "Waiting for map and FMS path to be received");
    return;
  }

  if (fms_path_msg_.poses.empty()) {
    RCLCPP_WARN(this->get_logger(), "Received path is empty");
    return;
  }

  RCLCPP_INFO(this->get_logger(), "FMS path valid, publishing it directly");
  planned_path_pub_->publish(fms_path_msg_); 
}

bool GlobalPlanner::isPathValid(const nav_msgs::msg::Path& path)
{
  if (path.poses.empty()) {
    RCLCPP_INFO(this->get_logger(), "[isPathValid] Return false: path empty");
    return false;
  }

  auto checkValidPointWithMargin = [&](const geometry_msgs::msg::Point& pt) -> bool {
    int mx, my;
    if (!worldToMap(pt, mx, my)) {
      RCLCPP_INFO(this->get_logger(), "[isPathValid] Return false: point outside map bounds");
      return false;
    }

    int margin_cells = std::max(1, static_cast<int>(min_safe_distance_ / map_msg_->info.resolution));

    for (int dy = -margin_cells; dy <= margin_cells; ++dy) {
      int y = my + dy;
      if (y < 0 || y >= static_cast<int>(map_msg_->info.height)) continue;

      for (int dx = -margin_cells; dx <= margin_cells; ++dx) {
        int x = mx + dx;
        if (x < 0 || x >= static_cast<int>(map_msg_->info.width)) continue;

        int idx = y * map_msg_->info.width + x;
        int val = map_msg_->data[idx];

        if (val == -1 && !allow_unknown_area_) {
          RCLCPP_INFO(this->get_logger(), "[isPathValid] Return false: unknown cell");
          return false;
        }

        if (val >= 50) {
          RCLCPP_INFO(this->get_logger(), "[isPathValid] Return false: obstacle detected at (%d, %d)", x, y);
          return false;
        }
      }
    }
    return true;
  };

  // 시작/목표 포인트 반드시 검사
  if (!checkValidPointWithMargin(path.poses.front().pose.position)) {
    RCLCPP_INFO(this->get_logger(), "[isPathValid] Return false: start point invalid");
    return false;
  }
  if (!checkValidPointWithMargin(path.poses.back().pose.position)) {
    RCLCPP_INFO(this->get_logger(), "[isPathValid] Return false: goal point invalid");
    return false;
  }

  // 중간 점은 간격을 두고 검사
  for (size_t i = 0; i < path.poses.size(); i += path_point_sampling_interval_) {
    if (!checkValidPointWithMargin(path.poses[i].pose.position)) {
      RCLCPP_INFO(this->get_logger(), "[isPathValid] Return false: invalid point at index %zu", i);
      return false;
    }
  }

  // 마지막 점 검사 (샘플링 누락 대비)
  if ((path.poses.size() - 1) % path_point_sampling_interval_ != 0) {
    if (!checkValidPointWithMargin(path.poses.back().pose.position)) {
      RCLCPP_INFO(this->get_logger(), "[isPathValid] Return false: invalid last point");
      return false;
    }
  }

  RCLCPP_INFO(this->get_logger(), "[isPathValid] Path is valid");
  return true;
}

void GlobalPlanner::planAndPublishAstarPath()
{
  std::vector<GridPoint> astar_path = planAstarPath();

  RCLCPP_INFO(this->get_logger(), "A* path length: %zu", astar_path.size());

  if (astar_path.empty()) {
    RCLCPP_WARN(this->get_logger(), "A* planning failed: path is empty");
    return;
  }

  nav_msgs::msg::Path path_msg;
  path_msg.header.frame_id = map_msg_->header.frame_id;
  path_msg.header.stamp = this->now();

  for (const auto& p : astar_path) {
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header = path_msg.header;
    pose_stamped.pose.position.x = map_msg_->info.origin.position.x + (p.x + 0.5) * map_msg_->info.resolution;
    pose_stamped.pose.position.y = map_msg_->info.origin.position.y + (p.y + 0.5) * map_msg_->info.resolution;
    pose_stamped.pose.position.z = 0.0;
    pose_stamped.pose.orientation.w = 1.0;
    path_msg.poses.push_back(pose_stamped);
  }

  planned_path_pub_->publish(path_msg);
  RCLCPP_INFO(this->get_logger(), "Published A* planned path with %zu points", astar_path.size());
}

std::vector<GridPoint> GlobalPlanner::planAstarPath()
{
  if (!map_msg_) {
    RCLCPP_WARN(this->get_logger(), "No map available for planning");
    return {};
  }

  if (odom_received_) {
    start_ = current_odom_.pose.pose;
  } else {
    // 실물이 없으니 더미 좌표 사용
    start_.position.x = 1.0;
    start_.position.y = 1.0;
    start_.orientation.w = 1.0;
    RCLCPP_WARN(this->get_logger(), "No odometry data, using dummy start position");
  }

  goal_ = fms_path_msg_.poses.back().pose;
 
  RCLCPP_INFO(this->get_logger(), "Start pose set to: x=%.3f, y=%.3f",
              start_.position.x, start_.position.y);
  RCLCPP_INFO(this->get_logger(), "Goal pose set to: x=%.3f, y=%.3f",
              goal_.position.x, goal_.position.y);
  
  int start_x, start_y, goal_x, goal_y;

  if (!worldToMap(start_.position, start_x, start_y) ||
      !worldToMap(goal_.position, goal_x, goal_y))
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to convert start/goal to map coordinates");
    return {};
  }

  return astar_planner_.planPath({start_x, start_y}, {goal_x, goal_y});
}

/*
  월드 좌표(world_pt)를 맵 인덱스(mx, my)로 변환.
  - resolution : 셀 크기 (m/셀)
  - origin     : 맵 (0,0) 셀의 월드 좌표
  - 변환 후 인덱스가 맵 범위 밖이면 false 반환
*/
bool GlobalPlanner::worldToMap(const geometry_msgs::msg::Point& world_pt, int& mx, int& my)
{
  if (!map_msg_) {
    RCLCPP_WARN(this->get_logger(), "No map available for coordinate conversion");
    return false;
  }

  double origin_x = map_msg_->info.origin.position.x;
  double origin_y = map_msg_->info.origin.position.y;
  double resolution = map_msg_->info.resolution;

  // world_pt.x 위치에서 맵 원점(origin_x)을 빼서 원점 기준 상대 위치를 구함
  // 이 값을 resolution(셀 크기)로 나누면 해당 좌표가 몇 번째 셀에 해당하는지 알 수 있음
  // 같은 방식으로 y 좌표도 계산
  mx = static_cast<int>((world_pt.x - origin_x) / resolution);
  my = static_cast<int>((world_pt.y - origin_y) / resolution);

  if (mx < 0 || my < 0 ||
      mx >= static_cast<int>(map_msg_->info.width) ||
      my >= static_cast<int>(map_msg_->info.height))
  {
    RCLCPP_WARN(this->get_logger(), 
                "Point (%.2f, %.2f) is out of map bounds", 
                world_pt.x, world_pt.y);
    return false;
  }
  return true;
}

bool GlobalPlanner::isValid(const GridPoint& p)
{
  if (!map_msg_) return false;
  if (p.x < 0 || p.x >= static_cast<int>(map_msg_->info.width)) return false;
  if (p.y < 0 || p.y >= static_cast<int>(map_msg_->info.height)) return false;
  return true;
}

void GlobalPlanner::saveFmsPathToFile()
{
    if (fms_path_msg_.poses.empty()) {
        RCLCPP_WARN(this->get_logger(), "No FMS path to save.");
        return;
    }

    // 현재 시간 기준 파일명 생성
    std::time_t t = std::time(nullptr);
    std::tm tm;
    localtime_r(&t, &tm); // 스레드 안전 함수 사용
    std::stringstream ss;
    ss << "/home/ysh/fms_path_"
       << std::put_time(&tm, "%Y%m%d_%H%M%S") << ".csv";
    std::string filename = ss.str();

    std::ofstream file(filename);
    if (!file.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open file: %s", filename.c_str());
        return;
    }

    // CSV 헤더
    file << "x,y\n";

    // 경로 포인트 기록
    for (const auto& pose_stamped : fms_path_msg_.poses) {
        file << pose_stamped.pose.position.x << ","
             << pose_stamped.pose.position.y << "\n";
    }

    file.close();
    RCLCPP_INFO(this->get_logger(), "FMS path saved to %s", filename.c_str());
}
