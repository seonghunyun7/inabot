#include "aruco_qr_localizer.hpp"
#include <sstream>

ArucoLocalizer::ArucoLocalizer(const rclcpp::NodeOptions& options)
: Node("aruco_localizer", options), camera_info_received_(false)
{
    loadParameters();

    // OpenCV 4.5.4
    // Detector parameters 생성
    detector_params_ = cv::aruco::DetectorParameters::create();
    // Dictionary 생성
    dictionary_ = cv::aruco::getPredefinedDictionary(aruco_dictionary_id_); //aruco_dictionary_id: 6 # DICT_5X5_250

	auto qos = rclcpp::QoS(1).best_effort();
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        image_topic_, /* qos*/ 10, std::bind(&ArucoLocalizer::imageCallback, this, std::placeholders::_1)
    );

	camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
		camera_info_topic_, /*qos*/10, std::bind(&ArucoLocalizer::cameraInfoCallback, this, std::placeholders::_1)
    );

	// Publishers
	_image_pub = this->create_publisher<sensor_msgs::msg::Image>("/aruco_image_proc", qos);
    poses_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("aruco_poses", 10);
    markers_pub_ = this->create_publisher<inabot_msgs::msg::ArucoMarkers>("aruco_markers", 10); // 변경
}

void ArucoLocalizer::loadParameters()
{
    this->declare_parameter<double>("marker_size", 0.05);
    this->declare_parameter<int>("aruco_id", 0);
    this->declare_parameter<int>("aruco_dictionary_id", 6); //DICT_5X5_250
    this->declare_parameter<std::string>("image_topic", "/camera/image_raw");
    this->declare_parameter<std::string>("camera_info_topic", "/camera/camera_info");
    this->declare_parameter<std::string>("camera_frame", "camera_color_frame");
    this->declare_parameter<std::string>("qr_topic", "/camera/color/qr_image_raw");
 
    this->get_parameter("marker_size", marker_size_);
    this->get_parameter("aruco_id", aruco_id_);
    this->get_parameter("aruco_dictionary_id", aruco_dictionary_id_);
    this->get_parameter("image_topic", image_topic_);
    this->get_parameter("camera_info_topic", camera_info_topic_);
    this->get_parameter("camera_frame", camera_frame_);

    RCLCPP_INFO(this->get_logger(), "marker_size: %f", marker_size_);
    RCLCPP_INFO(this->get_logger(), "aruco_id: %d", aruco_id_);
    RCLCPP_INFO(this->get_logger(), "aruco_dictionary_id: %d", aruco_dictionary_id_);
}

void ArucoLocalizer::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
    // Always update the camera matrix and distortion coefficients from the new message
    intrinsic_mat_ = cv::Mat(3, 3, CV_64F, const_cast<double*>(msg->k.data())).clone(); // Use clone to ensure a deep copy
    distortion_ = cv::Mat(msg->d.size(), 1, CV_64F, const_cast<double*>(msg->d.data())).clone(); // Use clone to ensure a deep copy

    camera_info_received_ = true;

   	// Log the first row of the camera matrix to verify correct values
	RCLCPP_INFO(get_logger(), "Camera matrix updated:\n[%f, %f, %f]\n[%f, %f, %f]\n[%f, %f, %f]",
		    intrinsic_mat_.at<double>(0, 0), intrinsic_mat_.at<double>(0, 1), intrinsic_mat_.at<double>(0, 2),
		    intrinsic_mat_.at<double>(1, 0), intrinsic_mat_.at<double>(1, 1), intrinsic_mat_.at<double>(1, 2),
		    intrinsic_mat_.at<double>(2, 0), intrinsic_mat_.at<double>(2, 1), intrinsic_mat_.at<double>(2, 2));
	RCLCPP_INFO(get_logger(), "Camera Matrix: fx=%f, fy=%f, cx=%f, cy=%f",
		    intrinsic_mat_.at<double>(0, 0), // fx
		    intrinsic_mat_.at<double>(1, 1), // fy
		    intrinsic_mat_.at<double>(0, 2), // cx
		    intrinsic_mat_.at<double>(1, 2)  // cy
		   );

	// Check if focal length is zero after update
	if (intrinsic_mat_.at<double>(0, 0) == 0) {
		RCLCPP_ERROR(get_logger(), "Focal length is zero after update!");

	} else {
		RCLCPP_INFO(get_logger(), "Updated camera intrinsics from camera_info topic.");

		RCLCPP_INFO(get_logger(), "Unsubscribing from camera info topic");
		camera_info_sub_.reset();
	}
}

/*
QR 처리
 - 이미지에서 QR 감지
 - 존재하면 로그 출력
 - 이미지에 사각형으로 표시
 - Pose 계산 하지 않음

ArUco 처리
- 이미지에서 ArUco 감지
- 존재하면 Pose 계산 및 로그 출력
- PoseArray와 마커 메시지에 추가
- 이미지에 축 그리기
*/
void ArucoLocalizer::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    if (!camera_info_received_) {
        RCLCPP_WARN(this->get_logger(), "Waiting for camera info...");
        return;
    }

    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat gray;
    cv::cvtColor(cv_ptr->image, gray, cv::COLOR_BGR2GRAY);

    #if __USDE_OPENCV_QR // opencv version 빌드해야 됨..
    // --- QR 감지 ---
    cv::QRCodeDetector qr_detector;
    std::string qr_data;
    std::vector<cv::Point> qr_bbox;
    qr_data = qr_detector.detectAndDecode(gray, qr_bbox);

    if (!qr_data.empty()) {
        RCLCPP_INFO(get_logger(), "QR detected: %s", qr_data.c_str());
        if (!qr_bbox.empty()) {
            for (size_t i = 0; i < qr_bbox.size(); i++) {
                cv::line(cv_ptr->image, qr_bbox[i], qr_bbox[(i+1)%qr_bbox.size()], cv::Scalar(0,255,0), 2);
            }
        }
    }
    #else
    // --- QR 감지 (ZXing) ---
    ZXing::ImageView image_view(gray.data, gray.cols, gray.rows, ZXing::ImageFormat::Lum);

    ZXing::ReaderOptions options;
    options.setFormats(ZXing::BarcodeFormat::QRCode);

    ZXing::Result result = ZXing::ReadBarcode(image_view, options);

    if (result.isValid()) {
        std::string qr_data = result.text();  // 이미 UTF-8
        RCLCPP_INFO(get_logger(), "QR detected: %s", qr_data.c_str());
    }
    #endif

    // --- ArUco 감지 ---
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners, rejected;
    cv::aruco::detectMarkers(gray, dictionary_, corners, ids, detector_params_, rejected);
    cv::aruco::drawDetectedMarkers(cv_ptr->image, corners, ids);

    geometry_msgs::msg::PoseArray pose_array;
    inabot_msgs::msg::ArucoMarkers markers_msg;
    pose_array.header = msg->header;
    markers_msg.header = msg->header;
    if (!camera_frame_.empty()) {
        pose_array.header.frame_id = camera_frame_;
        markers_msg.header.frame_id = camera_frame_;
    }

    if (!ids.empty()) {
        std::vector<cv::Vec3d> rvecs, tvecs;
        cv::aruco::estimatePoseSingleMarkers(corners, marker_size_, intrinsic_mat_, distortion_, rvecs, tvecs);

        for (size_t i = 0; i < ids.size(); i++) {
            #if _ONE_MARKER_ID_
            if (ids[i] != aruco_id_) continue;
            #endif

            geometry_msgs::msg::Pose pose;
            pose.position.x = tvecs[i][0];
            pose.position.y = tvecs[i][1];
            pose.position.z = tvecs[i][2];

            cv::Mat rot_mat;
            cv::Rodrigues(rvecs[i], rot_mat);
            tf2::Matrix3x3 tf_rot(
                rot_mat.at<double>(0,0), rot_mat.at<double>(0,1), rot_mat.at<double>(0,2),
                rot_mat.at<double>(1,0), rot_mat.at<double>(1,1), rot_mat.at<double>(1,2),
                rot_mat.at<double>(2,0), rot_mat.at<double>(2,1), rot_mat.at<double>(2,2)
            );
            tf2::Quaternion q;
            tf_rot.getRotation(q);
            pose.orientation = tf2::toMsg(q);

            pose_array.poses.push_back(pose);
            markers_msg.poses.push_back(pose);
            markers_msg.marker_ids.push_back(ids[i]);

            RCLCPP_INFO(this->get_logger(), "Detected Marker ID: %d", ids[i]);
        }
    }

    // Publish Pose (ArUco만)
    poses_pub_->publish(pose_array);
    markers_pub_->publish(markers_msg);

    #if 1 //__FOR_RVIZ_DEBUG
    // 이미지 발행 (QR + ArUco 같이 표시)
    cv_bridge::CvImage out_msg;
    out_msg.header = msg->header;
    out_msg.encoding = sensor_msgs::image_encodings::BGR8;
    out_msg.image = cv_ptr->image;
    _image_pub->publish(*out_msg.toImageMsg().get());
    #endif
}

void ArucoLocalizer::annotate_image(cv_bridge::CvImagePtr image, const cv::Vec3d& target)
{
	// Annotate the image with the target position and marker size
	std::ostringstream stream;
	stream << std::fixed << std::setprecision(2);
	stream << "X: "  << target[0] << " Y: " << target[1]  << " Z: " << target[2];
	std::string text_xyz = stream.str();

	int fontFace = cv::FONT_HERSHEY_SIMPLEX;
	double fontScale = 1;
	int thickness = 2;
	int baseline = 0;
	cv::Size textSize = cv::getTextSize(text_xyz, fontFace, fontScale, thickness, &baseline);
	baseline += thickness;
	cv::Point textOrg((image->image.cols - textSize.width - 10), (image->image.rows - 10));
	cv::putText(image->image, text_xyz, textOrg, fontFace, fontScale, cv::Scalar(0, 255, 255), thickness, 8);
}