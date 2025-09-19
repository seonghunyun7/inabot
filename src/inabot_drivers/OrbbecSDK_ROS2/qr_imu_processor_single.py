# qr_imu_processor_robot_frame.py

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

        # Camera + IMU 구독
        self.color_sub = self.create_subscription(Image, '/camera/color/image_raw', self.color_callback, 10)
        self.depth_sub = self.create_subscription(Image, '/camera/depth/image_raw', self.depth_callback, 10)
        self.accel_sub = self.create_subscription(Imu, '/camera/accel/imu_info', self.accel_callback, 10)
        self.gyro_sub = self.create_subscription(Imu, '/camera/gyro/imu_info', self.gyro_callback, 10)
        self.sample_sub = self.create_subscription(Imu, '/camera/gyro_accel/sample', self.sample_callback, 10)

        # QR 인식기
        self.qr_detector = cv2.QRCodeDetector()

        # 최신 데이터 저장
        self.latest_depth = None
        self.latest_imu = {'ax':0,'ay':0,'az':0,'gx':0,'gy':0,'gz':0}

    def color_callback(self, msg):
        color_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        data, bbox, _ = self.qr_detector.detectAndDecode(color_img)

        if bbox is not None and len(bbox[0]) > 0:
            # QR Bounding box 표시
            n = len(bbox[0])
            for i in range(n):
                pt1 = tuple(bbox[0][i].astype(int))
                pt2 = tuple(bbox[0][(i+1) % n].astype(int))
                cv2.line(color_img, pt1, pt2, (0, 255, 0), 2)

            # 중심점 계산
            cx = int(np.mean(bbox[0][:,0]))
            cy = int(np.mean(bbox[0][:,1]))

            # Depth에서 거리 계산 (주변 3x3 평균)
            distance = None
            if self.latest_depth is not None:
                h, w = self.latest_depth.shape
                if 1 <= cy < h-1 and 1 <= cx < w-1:
                    patch = self.latest_depth[cy-1:cy+2, cx-1:cx+2]
                    distance = np.mean(patch) / 1000.0  # mm -> m

            # IMU 기반 로봇 좌표계 변환 (단순 2D yaw 기준)
            x_robot, y_robot = None, None
            if distance is not None:
                # yaw 계산 (간단화, 실제 필요시 Madgwick/TF 적용 가능)
                gx, gy, gz = self.latest_imu['gx'], self.latest_imu['gy'], self.latest_imu['gz']
                # 여기서는 gyro.z를 yaw로 가정 (rad)
                yaw = gz
                # 카메라 기준 QR 좌표 (z축 전방)
                x_cam = 0
                y_cam = distance
                # 로봇 좌표계로 회전 변환
                x_robot = x_cam * np.cos(yaw) - y_cam * np.sin(yaw)
                y_robot = x_cam * np.sin(yaw) + y_cam * np.cos(yaw)

            # IMU 정보 문자열
            imu_info = f"Accel: ({self.latest_imu['ax']:.2f},{self.latest_imu['ay']:.2f},{self.latest_imu['az']:.2f}) " \
                       f"Gyro: ({self.latest_imu['gx']:.2f},{self.latest_imu['gy']:.2f},{self.latest_imu['gz']:.2f})"

            self.get_logger().info(f"QR Code: {data}, Distance: {distance:.3f} m, Robot Pos: ({x_robot},{y_robot}), {imu_info}")

        cv2.imshow("QR Detection", color_img)
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
