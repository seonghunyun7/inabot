#ifndef LIDAR_OBSTACLE_DETECTOR_HPP_
#define LIDAR_OBSTACLE_DETECTOR_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "laser_geometry/laser_geometry.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>  // for fromROSMsg
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <tf2_ros/create_timer_ros.h>

// TF2 및 메시지 필터 관련
#include <tf2_ros/message_filter.h>
#include <message_filters/subscriber.h>
#include "geometry_msgs/msg/twist.hpp"
#include <std_msgs/msg/float64.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <limits>
#include <visualization_msgs/msg/marker.hpp>
#include "vision_msgs/msg/bounding_box2_d.hpp"
#include "vision_msgs/msg/bounding_box2_d_array.hpp"

#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/point32.hpp"

class LidarObstacleDetector : public rclcpp::Node
{
public:
    explicit LidarObstacleDetector(const rclcpp::NodeOptions& options);

private:

    bool load_robot_polygon_param();
    void initParameters();

    void scanFrontCallback(const std::shared_ptr<const sensor_msgs::msg::LaserScan> scan);
    float checkObstacleDistance(const std::shared_ptr<const sensor_msgs::msg::LaserScan> scan);
    
    void removeOutliers(sensor_msgs::msg::LaserScan& scan, double distance_threshold);
    void filterPointCloud(const sensor_msgs::msg::PointCloud2& laser_cloud, sensor_msgs::msg::LaserScan& output_scan);
    bool inBox(const geometry_msgs::msg::Point& point);
#if 0
    void publishBoundingBoxMarker(const pcl::PointXYZ& obstacle_point, const std::string& frame_id);
#endif    
    //visualizer
    void publish_roi_polygon(const rclcpp::Time& stamp);
    void publish_robot_polygon(const rclcpp::Time& stamp);
#if 0
    void stop();
    void goBack();
    void turnLeft();
#endif
    // TF2
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // LaserScan 구독 및 필터
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::LaserScan>> front_laser_sub_;
    std::shared_ptr<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>> front_laser_notifier_;

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr roi_marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr robot_marker_pub_;

    // 퍼블리셔들
    rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr roi_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr roi_timer_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr obstacle_cmd_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr obstacle_dist_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

    laser_geometry::LaserProjection projector_;

    // 파라미터
    double min_x_, max_x_, min_y_, max_y_;
    double avoid_back_speed_;
    bool use_visual_;
    float robot_front_x_;

    std::vector<geometry_msgs::msg::Point32> polygon_points_;
};

#endif  // LIDAR_OBSTACLE_DETECTOR_HPP_
