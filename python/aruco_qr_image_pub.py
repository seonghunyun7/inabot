#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
import qrcode # pip3 install qrcode
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class ArucoQRPublisher(Node):
    def __init__(self):
        super().__init__('aruco_qr_publisher')

        # Parameters
        self.declare_parameter("aruco_id", 6)
        self.declare_parameter("marker_size", 200)
        self.declare_parameter("aruco_dictionary", "DICT_5X5_250")
        self.declare_parameter("camera_frame", "camera_color_frame")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("robot_id", "robot_001")

        self.aruco_id = self.get_parameter("aruco_id").value
        self.marker_size = self.get_parameter("marker_size").value
        self.aruco_dict_name = self.get_parameter("aruco_dictionary").value
        self.camera_frame = self.get_parameter("camera_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.robot_id = self.get_parameter("robot_id").value

        # Publishers
        self.image_pub = self.create_publisher(Image, "/camera/color/image_raw", 10)
        self.camera_info_pub = self.create_publisher(CameraInfo, "/camera/color/camera_info", 10)

        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        self.bridge = CvBridge()

        # Generate images
        self.aruco_image = self.generate_aruco()
        self.qr_image = self.generate_qr()

        # Toggle flag for alternating images
        self.show_aruco = True

        # Timer
        self.timer = self.create_timer(0.1, self.timer_callback)

    def generate_aruco(self):
        dictionary_id = getattr(cv2.aruco, self.aruco_dict_name)
        dictionary = cv2.aruco.Dictionary_get(dictionary_id)
        marker = np.zeros((self.marker_size, self.marker_size), dtype=np.uint8)
        cv2.aruco.drawMarker(dictionary, self.aruco_id, self.marker_size, marker, 1)
        border = int(self.marker_size * 0.25)
        return cv2.copyMakeBorder(marker, border, border, border, border, cv2.BORDER_CONSTANT, value=255)

    def generate_qr(self):
        qr = qrcode.QRCode(
            version=1,
            error_correction=qrcode.constants.ERROR_CORRECT_L,
            box_size=10,
            border=4,
        )
        qr.add_data(self.robot_id)
        qr.make(fit=True)
        img = qr.make_image(fill_color="black", back_color="white")
        img = np.array(img.convert("L"))
        return cv2.resize(img, (self.marker_size, self.marker_size))

    def create_camera_info(self):
        cam_info = CameraInfo()
        cam_info.header.frame_id = self.camera_frame
        cam_info.width = 2560
        cam_info.height = 1444
        fx = 1500.0
        fy = 1500.0
        cx = cam_info.width / 2.0
        cy = cam_info.height / 2.0
        cam_info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        cam_info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        cam_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        cam_info.distortion_model = "plumb_bob"
        return cam_info

    def timer_callback(self):
        now = self.get_clock().now().to_msg()

        # Toggle image between ArUco and QR
        self.show_aruco = not self.show_aruco
        img_to_publish = self.aruco_image if self.show_aruco else self.qr_image

        # Convert to BGR and publish
        img_bgr = cv2.cvtColor(img_to_publish, cv2.COLOR_GRAY2BGR)
        img_msg = self.bridge.cv2_to_imgmsg(img_bgr, encoding="bgr8")
        img_msg.header.stamp = now
        img_msg.header.frame_id = self.camera_frame
        self.image_pub.publish(img_msg)

        # Publish CameraInfo
        cam_info = self.create_camera_info()
        cam_info.header.stamp = now
        self.camera_info_pub.publish(cam_info)

        # Publish TF: base_link -> camera_color_frame
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = self.base_frame
        t.child_frame_id = self.camera_frame
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.2
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = ArucoQRPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
