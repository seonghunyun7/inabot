# qr_imu_processor_multi.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu
from cv_bridge import CvBridge
import cv2
import numpy as np

class MultiQRIMUProcessor(Node):
    def __init__(self):
        super().__init__('multi_qr_imu_processor')
        self.bridge = CvBridge()

        # Camera + IMU 구독
        self.color_sub = self.create_subscription(Image, '/camera/color/image_raw', self.color_callback, 10)
        self.depth_sub = self.create_subscription(Image, '/camera/depth/image_raw', self.depth_callback, 10)
        self.accel_sub = self.create_subscription(Imu, '/camera/accel/imu_info', self.accel_callback, 10)
        self.gyro_sub = self.create_subscription(Imu, '/camera/gyro/imu_info', self.gyro_callback, 10)

        # QR 인식기
        self.qr_detector = cv2.QRCodeDetector()

        # 최신 데이터
        self.latest_depth = None
        self.latest_imu = {'ax':0,'ay':0,'az':0,'gx':0,'gy':0,'gz':0}

    def color_callback(self, msg):
        color_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # 다중 QR 코드 검출
        retval, decoded_infos, points_list, _ = self.qr_detector.detectAndDecodeMulti(color_img)

        if retval:
            for idx, bbox in enumerate(points_list):
                if bbox is None:
                    continue

                # QR Bounding box 표시
                n = len(bbox)
                for i in range(n):
                    pt1 = tuple(bbox[i].astype(int))
                    pt2 = tuple(bbox[(i+1) % n].astype(int))
                    cv2.line(color_img, pt1, pt2, (0, 255, 0), 2)

                # 중심점 계산
                cx = int(np.mean(bbox[:,0]))
                cy = int(np.mean(bbox[:,1]))

                # Depth 계산 (3x3 평균)
                distance = None
                if self.latest_depth is not None:
                    h, w = self.latest_depth.shape
                    if 1 <= cy < h-1 and 1 <= cx < w-1:
                        patch = self.latest_depth[cy-1:cy+2, cx-1:cx+2]
                        distance = np.mean(patch) / 1000.0

                # IMU 기반 로봇 좌표 변환 (단순 2D yaw 기준)
                x_robot, y_robot = None, None
                if distance is not None:
                    yaw = self.latest_imu['gz']  # gyro z를 yaw로 가정
                    x_cam = 0
                    y_cam = distance
                    x_robot = x_cam * np.cos(yaw) - y_cam * np.sin(yaw)
                    y_robot = x_cam * np.sin(yaw) + y_cam * np.cos(yaw)

                # IMU 정보
                imu_info = f"Accel: ({self.latest_imu['ax']:.2f},{self.latest_imu['ay']:.2f},{self.latest_imu['az']:.2f}) " \
                           f"Gyro: ({self.latest_imu['gx']:.2f},{self.latest_imu['gy']:.2f},{self.latest_imu['gz']:.2f})"

                qr_data = decoded_infos[idx] if idx < len(decoded_infos) else "Unknown"
                self.get_logger().info(f"QR Code[{idx}]: {qr_data}, Distance: {distance:.3f} m, Robot Pos: ({x_robot},{y_robot}), {imu_info}")

        cv2.imshow("Multi QR Detection", color_img)
        cv2.waitKey(1)

    def depth_callback(self, msg):
        self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def accel_callback(self, msg):
        self.latest_imu['ax'] = msg.linear_acceleration.x
        self.latest_imu['ay'] = msg.linear_acceleration.y
        self.latest_imu['az'] = msg.linear_acceleration.z

    def gyro_callback(self, msg):
        self.latest_imu['gx'] = msg.angular_velocity.x
        self.latest_imu['gy'] = msg.angular_velocity.y
        self.latest_imu['gz'] = msg.angular_velocity.z

def main(args=None):
    rclpy.init(args=args)
    node = MultiQRIMUProcessor()
    rclpy.spin(node)
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
