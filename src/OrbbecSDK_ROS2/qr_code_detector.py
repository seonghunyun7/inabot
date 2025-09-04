# qr_code_detector.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class QRCodeDetectorNode(Node):
    def __init__(self):
        super().__init__('qr_code_detector')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image, '/camera/color/image_raw', self.image_callback, 10)
        self.detector = cv2.QRCodeDetector()

    def image_callback(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        data, bbox, _ = self.detector.detectAndDecode(img)

        if bbox is not None:
            n = len(bbox[0])
            for i in range(n):
                pt1 = tuple(bbox[0][i])
                pt2 = tuple(bbox[0][(i+1) % n])
                cv2.line(img, pt1, pt2, (0, 255, 0), 2)

        if data:
            self.get_logger().info(f"QR Code Detected: {data}")

        cv2.imshow("QR Detection", img)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = QRCodeDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

