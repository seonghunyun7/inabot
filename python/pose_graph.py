import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt

class PoseGraphNode(Node):
    def __init__(self):
        super().__init__('pose_graph_node')

        # /tracked_pose 구독 (PoseStamped)
        self.tracked_subscription = self.create_subscription(
            PoseStamped,
            '/tracked_pose',
            self.tracked_pose_callback,
            10)
        
        # /odom 구독 (Odometry)
        # QoS 정의
        odom_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_pose_callback,
            qos_profile=odom_qos
        )
        
        self.tracked_x = []
        self.tracked_y = []
        self.odom_x = []
        self.odom_y = []

        # 실시간 그래프 초기 설정
        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.tracked_line, = self.ax.plot([], [], 'b.-', label='Slam Pose')  # 파란색
        self.odom_line, = self.ax.plot([], [], 'r.-', label='Odom')             # 빨간색
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_title('Tracked Pose vs Odom Trajectory')
        self.ax.legend()
    
    def tracked_pose_callback(self, msg: PoseStamped):
        x = msg.pose.position.x
        y = msg.pose.position.y
        self.tracked_x.append(x)
        self.tracked_y.append(y)
        self.update_plot()
    
    def odom_pose_callback(self, msg: Odometry):
        # Odometry 메시지에서 위치 추출
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.odom_x.append(x)
        self.odom_y.append(y)
        self.update_plot()
    
    def update_plot(self):
        self.tracked_line.set_xdata(self.tracked_x)
        self.tracked_line.set_ydata(self.tracked_y)
        self.odom_line.set_xdata(self.odom_x)
        self.odom_line.set_ydata(self.odom_y)

        self.ax.relim()
        self.ax.autoscale_view()

        plt.draw()
        plt.pause(0.001)

def main(args=None):
    rclpy.init(args=args)
    node = PoseGraphNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
