#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include "inabot_msgs/msg/aruco_markers.hpp"  // 변경
#include <cv_bridge/cv_bridge.h>
#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// ZXing-C++ 헤더
#include <ZXing/ReadBarcode.h>
#include <ZXing/BarcodeFormat.h>
#include <ZXing/TextUtfEncoding.h>

class ArucoLocalizer : public rclcpp::Node
{
public:
   explicit ArucoLocalizer(const rclcpp::NodeOptions& options);

private:
    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    void annotate_image(cv_bridge::CvImagePtr image, const cv::Vec3d& target);
    void loadParameters();

    //from camera
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr qr_sub_;  // QR 구독

    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr poses_pub_;
    rclcpp::Publisher<inabot_msgs::msg::ArucoMarkers>::SharedPtr markers_pub_; // 변경
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _image_pub;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr qr_pub_; // QR 발행용

    cv::Mat intrinsic_mat_;
    cv::Mat distortion_;
    bool camera_info_received_;

    cv::Ptr<cv::aruco::Dictionary> dictionary_;
    cv::Ptr<cv::aruco::DetectorParameters> detector_params_;
    
    double marker_size_;
    int aruco_id_;
    int aruco_dictionary_id_;
    std::string image_topic_;
    std::string camera_info_topic_;
    std::string camera_frame_;
};
