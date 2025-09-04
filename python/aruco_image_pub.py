#!/usr/bin/env python3
"""
ArUco marker generator and publisher as a virtual camera with TF.

- Generates an ArUco marker PNG (with border).
- Publishes it as /camera/color/image_raw (sensor_msgs/Image).
- Publishes a dummy /camera/color/camera_info (sensor_msgs/CameraInfo).
- Publishes TF from base_link -> camera_color_frame and camera_color_frame -> aruco_marker_<ID>.

Author: adapted for ROS2 Humble
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
import os

from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from math import radians, sin, cos

# --- CameraInfo 생성 ---
def create_dummy_camera_info(frame_id="camera_color_frame"):
    cam_msg = CameraInfo()
    cam_msg.header.frame_id = frame_id
    cam_msg.width = 2560
    cam_msg.height = 1444

    # 가상 초점거리
    fx = 1500.0
    fy = 1500.0
    cx = cam_msg.width / 2.0
    cy = cam_msg.height / 2.0

    cam_msg.k = [
        fx, 0.0, cx,
        0.0, fy, cy,
        0.0, 0.0, 1.0
    ]
    cam_msg.p = [
        fx, 0.0, cx, 0.0,
        0.0, fy, cy, 0.0,
        0.0, 0.0, 1.0, 0.0
    ]
    cam_msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
    cam_msg.distortion_model = "plumb_bob"
    return cam_msg

class ArucoImagePublisher(Node):
    def __init__(self):
        super().__init__('aruco_image_pub')

        # Parameters
        self.declare_parameter("aruco_id", 6)
        self.declare_parameter("marker_size", 200)
        self.declare_parameter("aruco_dictionary_id", "DICT_5X5_250")
        self.declare_parameter("camera_frame", "camera_color_frame")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("marker_z", 0.5)  # 마커 높이 (m)

        self.aruco_id = self.get_parameter("aruco_id").get_parameter_value().integer_value
        self.marker_px = self.get_parameter("marker_size").get_parameter_value().integer_value
        self.dictionary_name = self.get_parameter("aruco_dictionary_id").get_parameter_value().string_value
        self.camera_frame = self.get_parameter("camera_frame").get_parameter_value().string_value
        self.base_frame = self.get_parameter("base_frame").get_parameter_value().string_value
        self.marker_z = self.get_parameter("marker_z").get_parameter_value().double_value

        # Create ArUco dictionary and marker
        dictionary_id = getattr(cv2.aruco, self.dictionary_name)
        dictionary = cv2.aruco.Dictionary_get(dictionary_id)
        marker_image = np.zeros((self.marker_px, self.marker_px), dtype=np.uint8)
        cv2.aruco.drawMarker(dictionary, self.aruco_id, self.marker_px, marker_image, 1)

        # Add white border
        border_size = int(self.marker_px * 0.25)
        image_with_border = cv2.copyMakeBorder(marker_image, border_size, border_size, border_size, border_size,
                                               cv2.BORDER_CONSTANT, value=255)

        # Save PNG
        self.image_path = os.path.abspath(f"marker_{self.aruco_id:04d}.png")
        success = cv2.imwrite(self.image_path, image_with_border)
        if success:
            self.get_logger().info(f"Generated ArUco marker PNG with border: {self.image_path}")
        else:
            self.get_logger().error(f"Failed to save marker image at {self.image_path}")

        # ROS publishers
        self.bridge = CvBridge()
        self.image_pub = self.create_publisher(Image, "/camera/color/image_raw", 10)
        self.camera_info_pub = self.create_publisher(CameraInfo, "/camera/color/camera_info", 10)

        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Timer
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.publish_image = image_with_border

    def timer_callback(self):
        # Publish image
        image_bgr = cv2.cvtColor(self.publish_image, cv2.COLOR_GRAY2BGR)
        msg = self.bridge.cv2_to_imgmsg(image_bgr, encoding="bgr8")
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.camera_frame
        self.image_pub.publish(msg)

        # Publish camera info
        cam_msg = create_dummy_camera_info(self.camera_frame)
        cam_msg.header.stamp = msg.header.stamp
        self.camera_info_pub.publish(cam_msg)

        # Publish TF: base_link -> camera_color_frame
        t_cam = TransformStamped()
        t_cam.header.stamp = msg.header.stamp
        t_cam.header.frame_id = self.base_frame
        t_cam.child_frame_id = self.camera_frame
        t_cam.transform.translation.x = 0.0
        t_cam.transform.translation.y = 0.0
        t_cam.transform.translation.z = 0.2  # 카메라 높이
        t_cam.transform.rotation.x = 0.0
        t_cam.transform.rotation.y = 0.0
        t_cam.transform.rotation.z = 0.0
        t_cam.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t_cam)

def main(args=None):
    rclpy.init(args=args)
    node = ArucoImagePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
