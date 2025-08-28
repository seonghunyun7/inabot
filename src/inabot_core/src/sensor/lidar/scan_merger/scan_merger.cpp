#include "scan_merger.hpp"

scanMerger::scanMerger(const rclcpp::NodeOptions& options)
: Node("laser_scan_merger", options)
{
    tolerance_ = this->declare_parameter("transform_tolerance", 0.01);

    initialize_params();
    refresh_params();

#if defined(use_time_synchronizer)
    rclcpp::QoS qos = rclcpp::SensorDataQoS();
    laser1_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::LaserScan>>(this, topic1_, qos.get_rmw_qos_profile());
    laser2_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::LaserScan>>(this, topic2_, qos.get_rmw_qos_profile());
    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(10), *laser1_sub_, *laser2_sub_);
    sync_->registerCallback(std::bind(&scanMerger::synchronized_callback, this, std::placeholders::_1, std::placeholders::_2));
#else
    auto default_qos = rclcpp::SensorDataQoS();
    sub1_ = this->create_subscription<sensor_msgs::msg::LaserScan>(topic1_, default_qos, std::bind(&scanMerger::scan_callback1, this, std::placeholders::_1));
    sub2_ = this->create_subscription<sensor_msgs::msg::LaserScan>(topic2_, default_qos, std::bind(&scanMerger::scan_callback2, this, std::placeholders::_1));
#endif    
 
    laser_scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(integratedTopic_, rclcpp::SensorDataQoS().keep_last(10));
    occupancy_grid_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("integrated_occupancy_grid", 1);

    tf2_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
        this->get_node_base_interface(), this->get_node_timers_interface());
    tf2_->setCreateTimerInterface(timer_interface);
    tf2_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf2_);
}

// ------------------- Parameter Handling -------------------
void scanMerger::initialize_params() {
    this->declare_parameter<std::string>("integrated_topic");
    this->declare_parameter<std::string>("integrated_frame_id");

    this->declare_parameter<std::string>("scan_topic_1");
    this->declare_parameter<float>("laser_1_x_off");
    this->declare_parameter<float>("laser_1_y_off");
    this->declare_parameter<float>("laser_1_yaw");
    this->declare_parameter<bool>("show_1");

    this->declare_parameter<std::string>("scan_topic_2");
    this->declare_parameter<float>("laser_2_x_off");
    this->declare_parameter<float>("laser_2_y_off");
    this->declare_parameter<float>("laser_2_yaw");
    this->declare_parameter<bool>("show_2");

    this->declare_parameter<float>("robot_front_end");
    this->declare_parameter<float>("robot_rear_end");
    this->declare_parameter<float>("robot_right_end");
    this->declare_parameter<float>("robot_left_end");

    this->declare_parameter<float>("remove_radius", 0.2);
    this->declare_parameter<bool>("use_time_synchronizer", true);
    this->declare_parameter<bool>("publish_occupancy_grid", false);
}

void scanMerger::refresh_params() {
    this->get_parameter_or<std::string>("integrated_topic", integratedTopic_, "integrated_scan");
    this->get_parameter_or<std::string>("integrated_frame_id", integratedFrameId_, "laser");
    this->get_parameter_or<std::string>("scan_topic_1", topic1_, "lidar_front_right/scan");
    this->get_parameter_or<float>("laser_1_x_off", laser1XOff_, 0.0);
    this->get_parameter_or<float>("laser_1_y_off", laser1YOff_, 0.0);
    this->get_parameter_or<float>("laser_1_yaw", laser1Yaw_, 0.0);
    this->get_parameter_or<bool>("show_1", show1_, true);

    this->get_parameter_or<std::string>("scan_topic_2", topic2_, "lidar_rear_left/scan");
    this->get_parameter_or<float>("laser_2_x_off", laser2XOff_, 0.0);
    this->get_parameter_or<float>("laser_2_y_off", laser2YOff_, 0.0);
    this->get_parameter_or<float>("laser_2_yaw", laser2Yaw_, 0.0);
    this->get_parameter_or<bool>("show_2", show2_, false);

    this->get_parameter_or<float>("robot_front_end", robotFrontEnd_, 0.0);
    this->get_parameter_or<float>("robot_rear_end", robotRearEnd_, 0.0);
    this->get_parameter_or<float>("robot_right_end", robotRightEnd_, 0.0);
    this->get_parameter_or<float>("robot_left_end", robotLeftEnd_, 0.0);

    this->get_parameter_or<float>("remove_radius", remove_radius_, 0.2);

    // 추가: time synchronizer 사용 여부
    this->get_parameter_or<bool>("use_time_synchronizer", use_time_synchronizer_, true);
    this->get_parameter_or<bool>("publish_occupancy_grid", publish_occupancy_grid_, false);

    RCLCPP_INFO(this->get_logger(), "Parameters loaded:");
    RCLCPP_INFO(this->get_logger(), "integrated_topic: %s", integratedTopic_.c_str());
    RCLCPP_INFO(this->get_logger(), "integrated_frame_id: %s", integratedFrameId_.c_str());
    RCLCPP_INFO(this->get_logger(), "scan_topic_1: %s", topic1_.c_str());
    RCLCPP_INFO(this->get_logger(), "laser_1_x_off: %.3f", laser1XOff_);
    RCLCPP_INFO(this->get_logger(), "laser_1_y_off: %.3f", laser1YOff_);
    RCLCPP_INFO(this->get_logger(), "laser_1_yaw: %.3f", laser1Yaw_);
    RCLCPP_INFO(this->get_logger(), "show_1: %s", show1_ ? "true" : "false");

    RCLCPP_INFO(this->get_logger(), "scan_topic_2: %s", topic2_.c_str());
    RCLCPP_INFO(this->get_logger(), "laser_2_x_off: %.3f", laser2XOff_);
    RCLCPP_INFO(this->get_logger(), "laser_2_y_off: %.3f", laser2YOff_);
    RCLCPP_INFO(this->get_logger(), "laser_2_yaw: %.3f", laser2Yaw_);
    RCLCPP_INFO(this->get_logger(), "show_2: %s", show2_ ? "true" : "false");

    RCLCPP_INFO(this->get_logger(), "robot_front_end: %.3f", robotFrontEnd_);
    RCLCPP_INFO(this->get_logger(), "robot_rear_end: %.3f", robotRearEnd_);
    RCLCPP_INFO(this->get_logger(), "robot_right_end: %.3f", robotRightEnd_);
    RCLCPP_INFO(this->get_logger(), "robot_left_end: %.3f", robotLeftEnd_);

    RCLCPP_INFO(this->get_logger(), "remove_radius: %.3f", remove_radius_);
    RCLCPP_INFO(this->get_logger(), "use_time_synchronizer: %s", use_time_synchronizer_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "publish_occupancy_grid: %s", publish_occupancy_grid_ ? "true" : "false");
}

// ------------------- Callbacks -------------------
#if !defined(use_time_synchronizer)
void scanMerger::scan_callback1(const sensor_msgs::msg::LaserScan::SharedPtr _msg)
{
    laser1_ = _msg;
    if (laser2_) update_point_cloud_rgb();
}

void scanMerger::scan_callback2(const sensor_msgs::msg::LaserScan::SharedPtr _msg)
{
    laser2_ = _msg;
}
#else
void scanMerger::synchronized_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr &laser1_msg,
                                      const sensor_msgs::msg::LaserScan::ConstSharedPtr &laser2_msg)
{
    laser1_ = laser1_msg;
    laser2_ = laser2_msg;
    update_point_cloud_rgb();
}
#endif
// ------------------- Utility Functions -------------------
float scanMerger::GET_R(float x, float y) { return std::sqrt(x*x + y*y); }
float scanMerger::GET_THETA(float x, float y) { return std::atan2(y, x); }

float scanMerger::normalizeAngle(float angle) {
    while (angle > M_PI) angle -= 2 * M_PI;
    while (angle < -M_PI) angle += 2 * M_PI;
    return angle;
}

float scanMerger::interpolate(float angle_1, float angle_2, float magnitude_1, float magnitude_2, float current_angle) {
    if (angle_1 == angle_2) return magnitude_1;
    return magnitude_1 + (current_angle - angle_1) * ((magnitude_2 - magnitude_1) / (angle_2 - angle_1));
}

float scanMerger::selectMin(float magnitude_1, float magnitude_2) {
    if (std::isnan(magnitude_1)) return magnitude_2;
    if (std::isnan(magnitude_2)) return magnitude_1;
    return std::min(magnitude_1, magnitude_2);
}

void scanMerger::geometry_quat_to_rpy(double* roll, double* pitch, double* yaw, geometry_msgs::msg::Quaternion geometry_quat){
    tf2::Quaternion quat;
    tf2::convert(geometry_quat, quat);
    tf2::Matrix3x3(quat).getRPY(*roll, *pitch, *yaw);
}

void scanMerger::removePointsWithinRadius(std::vector<float>& ranges, float radius, float angle_min, float angle_increment)
{
    for (size_t i = 0; i < ranges.size(); ++i)
    {
        float angle = angle_min + i * angle_increment;
        float x = ranges[i] * cos(angle);
        float y = ranges[i] * sin(angle);
        float dist_sq = x * x + y * y;
        if (dist_sq < radius * radius) ranges[i] = std::numeric_limits<float>::infinity();
    }
}

// ------------------- update_point_cloud_rgb -------------------
void scanMerger::update_point_cloud_rgb()
{
    // TF가 준비되었는지 확인
    if (!tf2_->canTransform(integratedFrameId_, laser1_->header.frame_id, tf2::TimePointZero) ||
        !tf2_->canTransform(integratedFrameId_, laser2_->header.frame_id, tf2::TimePointZero))
    {
        RCLCPP_WARN(this->get_logger(), "TF not available yet: %s or %s",
                    laser1_->header.frame_id.c_str(), laser2_->header.frame_id.c_str());
        return;
    }

    try
    {
        trans1_ = tf2_->lookupTransform(integratedFrameId_, laser1_->header.frame_id, tf2::TimePointZero);
        trans2_ = tf2_->lookupTransform(integratedFrameId_, laser2_->header.frame_id, tf2::TimePointZero);
    }
    catch (tf2::TransformException &ex)
    {
        RCLCPP_WARN(this->get_logger(), "TF Exception: %s", ex.what());
        return;
    }

    double sensor1_r, sensor1_p, sensor1_y, sensor2_r, sensor2_p, sensor2_y;
    geometry_quat_to_rpy(&sensor1_r, &sensor1_p, &sensor1_y, trans1_.transform.rotation);
    geometry_quat_to_rpy(&sensor2_r, &sensor2_p, &sensor2_y, trans2_.transform.rotation);
    sensor1_y += laser1Yaw_;
    sensor2_y += laser2Yaw_;

    std::vector<std::array<float,2>> scan_data;
    int count = 0;
    float min_theta = 0;
    float max_theta = 0;
    if (show1_)
    {
        for (float i = laser1_->angle_min; i <= laser1_->angle_max; i += laser1_->angle_increment)
        {
            std::array<float, 2> pt;

            float laser_angle;
            if (fabs(sensor1_r) < M_PI / 2)
            {
                laser_angle = i;
            }
            else
            {
                laser_angle = -i;
            }
            //float intensity = laser1_->intensities[count]; // ★ intensity 값 추가
            float temp_x = laser1_->ranges[count] * std::cos(laser_angle);
            float temp_y = laser1_->ranges[count] * std::sin(laser_angle);
            pt[0] = temp_x * std::cos(sensor1_y) - temp_y * std::sin(sensor1_y);
            pt[0] += trans1_.transform.translation.x + laser1XOff_;
            pt[1] = temp_x * std::sin(sensor1_y) + temp_y * std::cos(sensor1_y);
            pt[1] += trans1_.transform.translation.y + laser1YOff_;
            count++;

            // 전방: pt[0] 값이 -10cm에서 10cm 사이
            // 후방: pt[0] 값이 -10cm에서 10cm 사이
            // 왼쪽: pt[1] 값이 -10cm에서 10cm 사이
            // 오른쪽: pt[1] 값이 -10cm에서 10cm 사이
            // ROI
            if (pt[0] < robotFrontEnd_ && pt[0] > -robotRearEnd_ && pt[1] < robotLeftEnd_ && pt[1] > -robotRightEnd_)
            {
                continue;
            }

            // 난반사 필터링: intensity가 너무 낮거나 높으면 무시
            // if (intensity <  10.0f  || intensity >  240.0f) {
            //    continue;
            //}

            float r_ = GET_R(pt[0], pt[1]);
            float theta_ = GET_THETA(pt[0], pt[1]);
            std::array<float, 2> res_;
            res_[1] = r_;
            res_[0] = theta_;
            scan_data.push_back(res_);
            if (theta_ < min_theta)
            {
                min_theta = theta_;
            }
            if (theta_ > max_theta)
            {
                max_theta = theta_;
            }
        }
    }

    count = 0;
    if (show2_)
    {
        for (float i = laser2_->angle_min; i <= laser2_->angle_max; i += laser2_->angle_increment)
        {
            std::array<float, 2> pt;

            float laser_angle;
            if (fabs(sensor2_r) < M_PI / 2)
            {
                laser_angle = i;
            }
            else
            {
                laser_angle = -i;
            }
            
            //float intensity = laser2_->intensities[count]; // ★ intensity 값 추가
            float temp_x = laser2_->ranges[count] * std::cos(laser_angle);
            float temp_y = laser2_->ranges[count] * std::sin(laser_angle);
            pt[0] = temp_x * std::cos(sensor2_y) - temp_y * std::sin(sensor2_y);
            pt[0] += trans2_.transform.translation.x + laser2XOff_;
            pt[1] = temp_x * std::sin(sensor2_y) + temp_y * std::cos(sensor2_y);
            pt[1] += trans2_.transform.translation.y + laser2YOff_;
            count++;

            if (pt[0] < robotFrontEnd_ && pt[0] > -robotRearEnd_ && pt[1] < robotLeftEnd_ && pt[1] > -robotRightEnd_)
            {
                continue;
            }

            // 난반사 필터링: intensity가 너무 낮거나 높으면 무시
            // if (intensity <  10.0f  || intensity >  240.0f) {
            //    continue;
            //}
            float r_ = GET_R(pt[0], pt[1]);
            float theta_ = GET_THETA(pt[0], pt[1]);
            std::array<float, 2> res_;
            res_[1] = r_;
            res_[0] = theta_;
            scan_data.push_back(res_);
            if (theta_ < min_theta)
            {
                min_theta = theta_;
            }
            if (theta_ > max_theta)
            {
                max_theta = theta_;
            }
        }
    }

    // 스캔 데이터 정렬
    std::sort(scan_data.begin(), scan_data.end(), [](std::array<float, 2> a, std::array<float, 2> b)
                { return a[0] < b[0]; });

    // 병합된 데이터의 포인트 개수 확인
    // RCLCPP_INFO(this->get_logger(), "Merged scan data size: %zu points", scan_data.size()); // 480
    auto integrated_msg_ = std::make_shared<sensor_msgs::msg::LaserScan>();
    integrated_msg_->header.frame_id = integratedFrameId_;
    integrated_msg_->header.stamp = laser1_->header.stamp;
    integrated_msg_->angle_min = min_theta;
    integrated_msg_->angle_max = max_theta;
    integrated_msg_->angle_increment = laser1_->angle_increment;
    integrated_msg_->time_increment = laser1_->time_increment;
    integrated_msg_->scan_time = laser1_->scan_time;
    integrated_msg_->range_min = laser1_->range_min;
    integrated_msg_->range_max = laser1_->range_max;
    size_t i = 1;
    std::vector<float> temp_range;
    for (float angle = min_theta; angle < max_theta; angle += laser1_->angle_increment)
    {
        // while (scan_data[i][0] < angle)
        while (i < scan_data.size() && scan_data[i][0] < angle)
        {
            i++;
        }

        // RCLCPP_INFO(this->get_logger(), "Angle: %f, scan_data[i][0]: %f, scan_data[i][1]: %f", angle, scan_data[i][0], scan_data[i][1]);

        // 거리 보간(interpolation) 및 필터링
        // 만약 두 거리(scan_data[i][1], scan_data[i-1][1])의 차이가 1.0m 이상이라면(급격한 변화, 난반사 가능성) 아예 값을 inf (장애물 없음)로 채웁니다.
        if (fabs(scan_data[i][1] - scan_data[i - 1][1]) < 0.5f                                                                                         // 범위 증가 20cm -> 50cm -> 30cm
            && (fabs(scan_data[i][0] - angle) < 1.5 * laser1_->angle_increment || fabs(scan_data[i - 1][0] - angle) < 1.5 * laser1_->angle_increment)) // angle increment의 1.5배
        {
            float range = interpolate(scan_data[i - 1][0], scan_data[i][0], scan_data[i - 1][1], scan_data[i][1], angle);
            // RCLCPP_INFO(this->get_logger(), "Interpolated range: %f",range);
            //  거리 변화가 급격한 경우 (난반사 가능성)
            if (fabs(scan_data[i][1] - scan_data[i - 1][1]) > 1.0f)
            {
                // 급격한 변화가 있는 데이터는 무시 (난반사일 가능성 있음)
                temp_range.push_back(std::numeric_limits<float>::infinity());
            }
            else
            {

                temp_range.push_back(range);
            }
        }
        else
        {
            temp_range.push_back(std::numeric_limits<float>::infinity());
        }
    }
    // 병합된 스캔 데이터의 개수 출력
    // RCLCPP_INFO(this->get_logger(), "Publishing integrated scan with %zu points", temp_range.size()); // 360
    // 필터링된 데이터 범위 내에서 데이터를 제거
    removePointsWithinRadius(temp_range, remove_radius_, min_theta, laser1_->angle_increment); // 20cm

    integrated_msg_->ranges = temp_range;
    laser_scan_pub_->publish(*integrated_msg_);
 
    // OccupancyGrid 발행
    if ( publish_occupancy_grid_ )
        publishOccupancyGrid(scan_data);
    ////////////
}

void scanMerger::publishOccupancyGrid(const std::vector<std::array<float,2>>& scan_data)
{
    nav_msgs::msg::OccupancyGrid occupancy_msg;
    occupancy_msg.header.frame_id = integratedFrameId_;
    occupancy_msg.header.stamp = this->now();

    // 동적 범위 계산
    float x_min = std::numeric_limits<float>::max();
    float x_max = std::numeric_limits<float>::lowest();
    float y_min = std::numeric_limits<float>::max();
    float y_max = std::numeric_limits<float>::lowest();

    for (const auto &pt : scan_data)
    {
        float r = pt[1];
        float theta = pt[0];
        if (!std::isfinite(r) || r <= 0.0f) continue;

        float x = r * cos(theta);
        float y = r * sin(theta);

        x_min = std::min(x_min, x);
        x_max = std::max(x_max, x);
        y_min = std::min(y_min, y);
        y_max = std::max(y_max, y);
    }

    float resolution = 0.05;
    occupancy_msg.info.resolution = resolution;
    occupancy_msg.info.width = static_cast<uint32_t>((x_max - x_min)/resolution) + 1;
    occupancy_msg.info.height = static_cast<uint32_t>((y_max - y_min)/resolution) + 1;
    occupancy_msg.info.origin.position.x = x_min;
    occupancy_msg.info.origin.position.y = y_min;
    occupancy_msg.info.origin.orientation.w = 1.0;
    occupancy_msg.data.resize(occupancy_msg.info.width * occupancy_msg.info.height, 0);

    // OccupancyGrid 데이터 채우기
    for (const auto &pt : scan_data)
    {
        float r = pt[1];
        float theta = pt[0];
        if (!std::isfinite(r) || r <= 0.0f) continue;

        int x_idx = static_cast<int>((r * cos(theta) - x_min) / resolution);
        int y_idx = static_cast<int>((r * sin(theta) - y_min) / resolution);

        if (x_idx >= 0 && x_idx < occupancy_msg.info.width &&
            y_idx >= 0 && y_idx < occupancy_msg.info.height)
        {
            occupancy_msg.data[y_idx * occupancy_msg.info.width + x_idx] = 100;
        }
    }

    occupancy_grid_pub_->publish(occupancy_msg);
}
