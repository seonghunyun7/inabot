import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import matplotlib.pyplot as plt
from collections import deque

class MotorLoadPlotter(Node):
    def __init__(self):
        super().__init__('motor_load_plotter')

        self.load_data = deque(maxlen=100)
        self.time_data = deque(maxlen=100)

        self.subscription = self.create_subscription(
            Float32,
            'motor_load',
            self.motor_load_callback,
            10)

        self.start_time = self.get_clock().now()

        self.fig, self.ax = plt.subplots()
        self.line, = self.ax.plot([], [], 'b-')
        self.ax.set_ylim(0, 10)
        self.ax.set_xlim(0, 10)
        self.ax.set_xlabel('Time (s)')
        self.ax.set_ylabel('Motor Load (%)')
        self.ax.set_title('Real-time Motor Load')

    def motor_load_callback(self, msg):
        current_time = self.get_clock().now()
        duration = current_time - self.start_time
        elapsed = duration.nanoseconds * 1e-9
        self.time_data.append(elapsed)

        scaled_load = msg.data * 100.0  # 부하율을 %로 변환
        self.load_data.append(scaled_load)

    def update_plot(self):
        if len(self.time_data) == 0:
            return

        xmin = self.time_data[-1] - 10 if self.time_data[-1] > 10 else 0
        xmax = self.time_data[-1]
        self.ax.set_xlim(xmin, xmax)

        # y축 자동 스케일링: 데이터 최대값 기준으로 약간 여유 둠
        max_load = max(self.load_data) if self.load_data else 10
        ylim_max = max(10, max_load * 1.2)  # 최소 10, 최대값의 120% 여유 공간
        self.ax.set_ylim(0, ylim_max)

        self.line.set_data(self.time_data, self.load_data)
        self.ax.relim()
        self.ax.autoscale_view(False, True, False)  # x축은 수동, y축은 자동 스케일링
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

def main(args=None):
    rclpy.init(args=args)
    node = MotorLoadPlotter()

    plt.ion()  # 인터랙티브 모드 ON
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
