import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
from collections import deque

class TrajectoryPlotter(Node):
    def __init__(self):
        super().__init__('trajectory_plotter')

        self.x_data = deque(maxlen=1000)  # x 좌표 저장 (최대 1000개)
        self.y_data = deque(maxlen=1000)  # y 좌표 저장

        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)

        self.fig, self.ax = plt.subplots()
        self.line, = self.ax.plot([], [], 'b-')
        self.ax.set_xlabel('X [m]')
        self.ax.set_ylabel('Y [m]')
        self.ax.set_title('Robot Trajectory')
        self.ax.axis('equal')  # x,y 축 비율 동일하게 유지
        self.ax.grid(True)

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        self.x_data.append(x)
        self.y_data.append(y)

    def update_plot(self):
        if len(self.x_data) == 0:
            return

        self.line.set_data(self.x_data, self.y_data)

        # 축 범위 자동 조절
        self.ax.relim()
        self.ax.autoscale_view()

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPlotter()

    plt.ion()  # interactive mode on
    plt.show()

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            node.update_plot()
            plt.pause(0.1)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
