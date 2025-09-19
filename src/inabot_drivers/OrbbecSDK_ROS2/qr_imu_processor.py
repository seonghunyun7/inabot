# qr_imu_processor.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu
from cv_bridge import CvBridge
import cv2
import numpy as np

class QRIMUProcessor(Node):
    def __init__(self):
        super().__init__('qr_imu_processor')
        self.bridge = CvBridge()
        self.color_sub = self.create_subscription(Image, '/camera/color/image_raw', self.color_callback, 10)
        self.depth_sub = self.create_subscription(Image, '/camera/depth/image_raw', self.depth_callback, 10)
        self.accel_sub = self.create_subscription(Imu, '/camera/accel/imu_info', self.accel_callback, 10)
        self.gyro_sub = self.create_subscription(Imu, '/camera/gyro/imu_info', self.gyro_callback, 10)
        self.sample_sub = self.create_subscription(Imu, '/camera/gyro_accel/sample', self.sample_callback, 10)

        self.qr_detector = cv2.QRCodeDetector()
        self.latest_depth = None
        self.latest_imu = None

    def color_callback(self, msg):
        color_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        data, bbox, _ = self.qr_detector.detectAndDecode(color_img)

        if bbox is not None:
            n = len(bbox[0])
            for i in range(n):
                pt1 = tuple(bbox[0][i].astype(int))
                pt2 = tuple(bbox[0][(i+1) % n].astype(int))
                cv2.line(color_img, pt1, pt2, (0, 255, 0), 2)

            cx = int(np.mean(bbox[0][:,0]))
            cy = int(np.mean(bbox[0][:,1]))

            distance = None
            if self.latest_depth is not None:
                h, w = self.latest_depth.shape
                if 0 <= cy < h and 0 <= cx < w:
                    distance = self.latest_depth[cy, cx] / 1000.0

            if self.latest_imu is not None:
                imu_info = f"Accel: ({self.latest_imu['ax']:.2f},{self.latest_imu['ay']:.2f},{self.latest_imu['az']:.2f}) " \
                           f"Gyro: ({self.latest_imu['gx']:.2f},{self.latest_imu['gy']:.2f},{self.latest_imu['gz']:.2f})"
            else:
                imu_info = "IMU: not received yet"

            self.get_logger().info(f"QR Code: {data}, Distance: {distance} m, {imu_info}")

        cv2.imshow("QR Detection", color_img)
        cv2.waitKey(1)

    def depth_callback(self, msg):
        self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def accel_callback(self, msg):
        if self.latest_imu is None:
            self.latest_imu = {}
        self.latest_imu['ax'] = msg.linear_acceleration.x
        self.latest_imu['ay'] = msg.linear_acceleration.y
        self.latest_imu['az'] = msg.linear_acceleration.z

    def gyro_callback(self, msg):
        if self.latest_imu is None:
            self.latest_imu = {}
        self.latest_imu['gx'] = msg.angular_velocity.x
        self.latest_imu['gy'] = msg.angular_velocity.y
        self.latest_imu['gz'] = msg.angular_velocity.z

    def sample_callback(self, msg):
        pass  # 필요시 처리

def main(args=None):
    rclpy.init(args=args)
    node = QRIMUProcessor()
    rclpy.spin(node)
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


