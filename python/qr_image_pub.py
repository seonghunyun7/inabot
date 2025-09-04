#!/usr/bin/env python3
"""
QR dummy publisher as virtual camera with TF.

- Generates a QR code image with robot info.
- Publishes it as ROS2 /camera/color/qr_image_raw.
- Publishes dummy /camera/color/camera_info.
- Publishes TF from base_link -> camera_color_frame.

Author: adapted for ROS2 Humble
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
import qrcode

from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

# --- CameraInfo 생성 ---
def create_dummy_camera_info(frame_id="camera_color_frame"):
    cam_msg = CameraInfo()
    cam_msg.header.frame_id = frame_id
    cam_msg.width = 2560
    cam_msg.height = 1444

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

# --- QR 코드 생성 ---
def generate_dummy_qr(data="robot_001", size=200):
    qr = qrcode.QRCode(
        version=1,
        error_correction=qrcode.constants.ERROR_CORRECT_L,
        box_size=10,
        border=4,
    )
    qr.add_data(data)
    qr.make(fit=True)
    img = qr.make_image(fill_color="black", back_color="white")
    img = np.array(img.convert("L"))  # PIL -> numpy grayscale
    img = cv2.resize(img, (size, size))
    return img

class QRPublisher(Node):
    def __init__(self):
        super().__init__('qr_pub_only')

        # Parameters
        self.declare_parameter("marker_size", 200)
        self.declare_parameter("camera_frame", "camera_color_frame")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("robot_id", "robot_001")
        self.declare_parameter("marker_z", 0.5)

        self.marker_px = self.get_parameter("marker_size").get_parameter_value().integer_value
        self.camera_frame = self.get_parameter("camera_frame").get_parameter_value().string_value
        self.base_frame = self.get_parameter("base_frame").get_parameter_value().string_value
        self.robot_id = self.get_parameter("robot_id").get_parameter_value().string_value
        self.marker_z = self.get_parameter("marker_z").get_parameter_value().double_value

        # Create QR image
        self.qr_image = generate_dummy_qr(self.robot_id, self.marker_px)

        # ROS publishers
        self.bridge = CvBridge()
        self.qr_pub = self.create_publisher(Image, "/camera/color/image_raw", 10)
        self.camera_info_pub = self.create_publisher(CameraInfo, "/camera/color/camera_info", 10)

        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Timer
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        # --- Publish QR image ---
        qr_bgr = cv2.cvtColor(self.qr_image, cv2.COLOR_GRAY2BGR)
        qr_msg = self.bridge.cv2_to_imgmsg(qr_bgr, encoding="bgr8")
        qr_msg.header.stamp = self.get_clock().now().to_msg()
        qr_msg.header.frame_id = self.camera_frame
        self.qr_pub.publish(qr_msg)

        # --- Publish CameraInfo ---
        cam_msg = create_dummy_camera_info(self.camera_frame)
        cam_msg.header.stamp = qr_msg.header.stamp
        self.camera_info_pub.publish(cam_msg)

        # --- Publish TF ---
        t_cam = TransformStamped()
        t_cam.header.stamp = qr_msg.header.stamp
        t_cam.header.frame_id = self.base_frame
        t_cam.child_frame_id = self.camera_frame
        t_cam.transform.translation.x = 0.0
        t_cam.transform.translation.y = 0.0
        t_cam.transform.translation.z = self.marker_z
        t_cam.transform.rotation.x = 0.0
        t_cam.transform.rotation.y = 0.0
        t_cam.transform.rotation.z = 0.0
        t_cam.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t_cam)

        # --- Decode QR code for debug ---
        detector = cv2.QRCodeDetector()
        data, bbox, _ = detector.detectAndDecode(self.qr_image)
        if data:
            self.get_logger().info(f"QR data read: {data}")

def main(args=None):
    rclpy.init(args=args)
    node = QRPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
