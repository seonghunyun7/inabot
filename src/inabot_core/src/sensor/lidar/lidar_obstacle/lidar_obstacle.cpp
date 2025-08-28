#include "sensor/lidar/lidar_obstacle/lidar_obstacle_detector.hpp"  

LidarObstacleDetector::LidarObstacleDetector(const rclcpp::NodeOptions& options)
  : Node("lidar_obstacle_detector", options)
{
    if (!load_robot_polygon_param())
    {
        RCLCPP_WARN(this->get_logger(), "Using default polygon shape.");

        geometry_msgs::msg::Point32 p1, p2, p3, p4;
        p1.x = 0.401f; p1.y = 0.27346f; p1.z = 0.0f;
        p2.x = 0.401f; p2.y = -0.27346f; p2.z = 0.0f;
        p3.x = -0.401f; p3.y = -0.27346f; p3.z = 0.0f;
        p4.x = -0.401f; p4.y = 0.27346f; p4.z = 0.0f;

        polygon_points_ = {p1, p2, p3, p4};
    }

    initParameters();

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
        this->get_node_base_interface(), this->get_node_timers_interface());
    
    tf_buffer_->setCreateTimerInterface(timer_interface);
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    auto default_qos = rclcpp::QoS(rclcpp::SensorDataQoS());
    default_qos.keep_last(10);

    front_laser_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::LaserScan>>(this, "/scan_front", default_qos.get_rmw_qos_profile());
    front_laser_notifier_ = std::make_shared<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>>(
        *front_laser_sub_, 
        *tf_buffer_, 
        "base_link", 
        10, 
        this->get_node_logging_interface(), 
        this->get_node_clock_interface());

    front_laser_notifier_->registerCallback(
                        std::bind(&LidarObstacleDetector::scanFrontCallback, this, std::placeholders::_1));
    front_laser_notifier_->setTolerance(rclcpp::Duration::from_seconds(0.05)); // Optional
 
    roi_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("roi_marker", 10);
    robot_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("robot_marker", 10);
    obstacle_dist_pub_ = this->create_publisher<std_msgs::msg::Float64>("/obstacle_forward_distance", 10);
#if 0
    obstacle_cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("avoidance/cmd_vel", 10);
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("marker_topic", 10);
#endif
}

void LidarObstacleDetector::initParameters()
{
    this->declare_parameter<double>("avoid_back_speed", -0.2);
    this->declare_parameter<bool>("use_visual", false);
    this->declare_parameter<double>("min_x", 0.0);
    this->declare_parameter<double>("max_x", 1.0);
    this->declare_parameter<double>("min_y", -0.6);
    this->declare_parameter<double>("max_y", 0.6);

    this->get_parameter("avoid_back_speed", avoid_back_speed_);
    this->get_parameter("use_visual", use_visual_);
    this->get_parameter("min_x", min_x_);
    this->get_parameter("max_x", max_x_);
    this->get_parameter("min_y", min_y_);
    this->get_parameter("max_y", max_y_);

    //min_x_ = 0.0;   // 전방 시작 (0m)
    //max_x_ = 1.0;   // 전방 끝 (1m)
    //min_y_ = -0.6;  // 좌측 끝 (-0.6m)
    //max_y_ = 0.6;   // 우측 끝 (0.6m)

    // 로봇 polygon에서 앞쪽 끝 x값 계산
    robot_front_x_ = -std::numeric_limits<float>::infinity();
    for (const auto& pt : polygon_points_) {
        if (pt.x > robot_front_x_) {
            robot_front_x_ = pt.x;
        }
    }

    // ROI를 로봇 앞 끝 기준으로 조정
    min_x_ += robot_front_x_;  // 로봇 앞 끝에서 시작
    max_x_ += robot_front_x_;  // 로봇 앞 끝에서 ROI 거리만큼 확장

    RCLCPP_INFO(this->get_logger(), "  use_visual_: %s", use_visual_ ? "ON" : "OFF");
    RCLCPP_INFO(this->get_logger(), "  min_x_: %.2f", min_x_);
    RCLCPP_INFO(this->get_logger(), "  max_x_: %.2f", max_x_);
    RCLCPP_INFO(this->get_logger(), "  min_y_: %.2f", min_y_);
    RCLCPP_INFO(this->get_logger(), "  max_y_: %.2f", max_y_);

    RCLCPP_INFO(this->get_logger(),"Parameter use_visual: %s",use_visual_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(),
            "ROI parameters: min_x=%.3f, max_x=%.3f, min_y=%.3f, max_y=%.3f",
            min_x_, max_x_, min_y_, max_y_);
}

bool LidarObstacleDetector::load_robot_polygon_param()
{
    this->declare_parameter<std::vector<double>>("polygon", std::vector<double>{});

    std::vector<double> raw_polygon;
    if (!this->get_parameter("polygon", raw_polygon))
        return false;

    if (raw_polygon.size() % 2 != 0) {
        RCLCPP_WARN(this->get_logger(), "Polygon parameter size must be even.");
        return false;
    }

    polygon_points_.clear();
    for (size_t i = 0; i < raw_polygon.size(); i += 2) {
        geometry_msgs::msg::Point32 pt;
        pt.x = raw_polygon[i];
        pt.y = raw_polygon[i + 1];
        pt.z = 0.0f;
        polygon_points_.push_back(pt);
    }

    // 첫 점을 마지막에 추가해서 닫기
    if (!polygon_points_.empty()) {
        polygon_points_.push_back(polygon_points_.front());
    }

    return true;
}

void LidarObstacleDetector::scanFrontCallback(const std::shared_ptr<const sensor_msgs::msg::LaserScan> scan)
{
    float dist = checkObstacleDistance(scan);

    // LiDAR 타임스탬프를 사용해서 ROI & 로봇 박스 퍼블리시
    publish_robot_polygon(scan->header.stamp);
    publish_roi_polygon(scan->header.stamp);
}

void LidarObstacleDetector::removeOutliers(sensor_msgs::msg::LaserScan& scan, double distance_threshold)
{
    int num_ranges = scan.ranges.size();

    for (int i = 1; i < num_ranges - 1; ++i)
    {
        if (!std::isfinite(scan.ranges[i]))
        {
            continue; // NaN 값은 건너뜁니다.
        }

        // 현재 포인트와 이전, 다음 포인트의 차이가 임계값을 넘는 경우 이상치로 간주
        if (std::isfinite(scan.ranges[i - 1]) && std::isfinite(scan.ranges[i + 1]))
        {
            if (fabs(scan.ranges[i] - scan.ranges[i - 1]) > distance_threshold &&
                fabs(scan.ranges[i] - scan.ranges[i + 1]) > distance_threshold)
            {
                scan.ranges[i] = std::numeric_limits<float>::quiet_NaN(); // 이상치는 NaN으로 설정
            }
        }
    }
}

// 전방 1m, 좌우 60cm 범위 설정
//min_x_ = 0.0;   // 전방 시작 (0m)
//max_x_ = 1.0;   // 전방 끝 (1m)
//min_y_ = -0.6;  // 좌측 끝 (-0.6m)
//max_y_ = 0.6;   // 우측 끝 (0.6m)
bool LidarObstacleDetector::inBox(const geometry_msgs::msg::Point& point)
{
    return point.x >= min_x_ && point.x <= max_x_ &&
           point.y >= min_y_ && point.y <= max_y_;
}

void LidarObstacleDetector::filterPointCloud(const sensor_msgs::msg::PointCloud2& laser_cloud, sensor_msgs::msg::LaserScan& output_scan)
{
    sensor_msgs::PointCloud2ConstIterator<uint32_t> index_it(laser_cloud, "index");
    sensor_msgs::PointCloud2ConstIterator<float> x_it(laser_cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> y_it(laser_cloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> z_it(laser_cloud, "z");

    for (; index_it != index_it.end(); ++index_it, ++x_it, ++y_it, ++z_it)
    {
        float x = *x_it;
        float y = *y_it;
        //float z = *z_it;  // z
        uint32_t index = *index_it;

        if (index >= output_scan.ranges.size()) continue;

        geometry_msgs::msg::Point point;
        point.x = x;
        point.y = y;
        point.z = 0.0;  // 2D LIDAR이므로 z 생략

        //포인트가 ROI(Box) 안에 들어오면 범위 값이 그대로 유지되므로 장애물로 인지
        //ROI 밖이면 NaN 처리되어 장애물로 인식되지 않음.
        if (!inBox(point))
        {
            output_scan.ranges[index] = std::numeric_limits<float>::quiet_NaN();
        }
    }
}

float LidarObstacleDetector::checkObstacleDistance(const std::shared_ptr<const sensor_msgs::msg::LaserScan> scan)
{ 
    sensor_msgs::msg::LaserScan scan_out = *scan;
    removeOutliers(scan_out, 0.5);

    // LaserScan → PointCloud2 변환
    sensor_msgs::msg::PointCloud2 cloud;
    try {
        projector_.transformLaserScanToPointCloud("base_link", *scan, cloud, *tf_buffer_);
    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN(this->get_logger(), "TF transform failed: %s", ex.what());
        return 0.0f;
    }
     
    // ROI 필터링
    // 전방 1m 좌우 60cm 만 라이더 데이터에 포함 + 로봇 전방 부터 + 50cm 더
    filterPointCloud(cloud, scan_out);

    // PointCloud2를 처리하여 base_link 기준으로 장애물 거리를 계산
    float min_distance = std::numeric_limits<float>::infinity();
    pcl::PointXYZ closest_point;

    for (size_t i = 0; i < scan_out.ranges.size(); ++i)
    {
        float r = scan_out.ranges[i];
        if (!std::isfinite(r) || r <= scan_out.range_min) continue;

        if (r < min_distance)
        {
            min_distance = r;
            float angle = scan_out.angle_min + i * scan_out.angle_increment;
            closest_point.x = r * std::cos(angle);
            closest_point.y = r * std::sin(angle);
            closest_point.z = 0.0f;
        }
    }

    std_msgs::msg::Float64 dist_msg;
    if (min_distance < std::numeric_limits<float>::infinity()) {
        dist_msg.data = min_distance;
    } else {
        dist_msg.data = 4.0; //default => 4m 인 경우 장애물이 없다.. 
    }

    obstacle_dist_pub_->publish(dist_msg);
    return (min_distance < std::numeric_limits<float>::infinity()) ? min_distance : 0.0f;
}

void LidarObstacleDetector::publish_roi_polygon(const rclcpp::Time& stamp)
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = stamp;
    marker.ns = "roi_polygon";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.scale.x = 0.02; // 선 두께
    marker.color.r = 1.0;  // 빨강
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    geometry_msgs::msg::Point p1, p2, p3, p4, p5;
    p1.x = max_x_; p1.y = max_y_;
    p2.x = max_x_; p2.y = min_y_;
    p3.x = min_x_; p3.y = min_y_;
    p4.x = min_x_; p4.y = max_y_;
    p5 = p1; // 닫기

    marker.points = {p1, p2, p3, p4, p5};
    roi_marker_pub_->publish(marker);
}

void LidarObstacleDetector::publish_robot_polygon(const rclcpp::Time& stamp)
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = stamp;
    marker.ns = "robot_polygon";
    marker.id = 1;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.scale.x = 0.02; // 선 두께
    marker.color.r = 0.0;
    marker.color.g = 0.0;  // 초록
    marker.color.b = 1.0;  // 파랑
    marker.color.a = 1.0;

    std::vector<geometry_msgs::msg::Point> points;
    points.reserve(polygon_points_.size());

    for (const auto& pt32 : polygon_points_) {
        geometry_msgs::msg::Point pt;
        pt.x = pt32.x;
        pt.y = pt32.y;
        pt.z = pt32.z;
        points.push_back(pt);
    }

    marker.points = points;
    robot_marker_pub_->publish(marker);
}

#if 0
void LidarObstacleDetector::publishBoundingBoxMarker(const pcl::PointXYZ& obstacle_point, const std::string& frame_id)
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id; //base_link
    marker.header.stamp = this->now();
    marker.ns = "obstacle";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = obstacle_point.x;
    marker.pose.position.y = obstacle_point.y;
    marker.pose.position.z = 0.1;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.1;
    marker.color.a = 0.8;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.lifetime = rclcpp::Duration::from_seconds(0.5);
    marker_pub_->publish(marker);
}

void LidarObstacleDetector::stop()
{
    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 0.0;
    obstacle_cmd_vel_pub_->publish(cmd_vel);
}

void LidarObstacleDetector::goBack()
{
    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = avoid_back_speed_;
    cmd_vel.angular.z = 0.0;
    obstacle_cmd_vel_pub_->publish(cmd_vel);
}

void LidarObstacleDetector::turnLeft()
{
    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 0.4363;
    obstacle_cmd_vel_pub_->publish(cmd_vel);
}
#endif