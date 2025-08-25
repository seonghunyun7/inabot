import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
import numpy as np
import matplotlib.pyplot as plt

class DummyMapPathAndPlannedPathNode(Node):
    def __init__(self):
        super().__init__('dummy_map_path_and_planned_path_node')

        # 퍼블리셔 (map, fms_planned_path)
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', 10)
        self.fms_path_pub = self.create_publisher(Path, '/fms_path', 10)

        # planned_path 구독
        self.planned_path_sub = self.create_subscription(
            Path, '/planned_path', self.planned_path_callback, 10)

        # 맵 관련 변수 설정
        self.resolution = 0.05  # 5cm 해상도
        self.width = 200        # 10m 공간 가로 셀 수
        self.height = 200       # 10m 공간 세로 셀 수

        # 맵 메시지 틀
        self.map_msg = OccupancyGrid()
        self.map_msg.header.frame_id = "map"
        self.map_msg.info.width = self.width
        self.map_msg.info.height = self.height
        self.map_msg.info.resolution = self.resolution
        self.map_msg.info.origin.position.x = 0.0
        self.map_msg.info.origin.position.y = 0.0

        # 장애물 생성 (예시)
        self.map_data = np.zeros((self.height, self.width), dtype=np.int8)
        self.map_data[30:70, 30:50] = 100
        self.map_data[100:140, 80:120] = 100
        self.map_data[140:160, 140:160] = 100  # 20x20 크기 장애물로 줄임
        #self.map_data[150:180, 150:190] = 100
        self.map_data[0:20, 120:160] = 100
        self.map_data[70:90, 0:40] = 100

        # 시작점 (장애물 없는 곳으로 설정)
        self.start = (1.0, 1.0)  # 1m, 1m (셀 기준 20,20)

        # 목표점 후보 좌표
        #self.goal = (7.5, 7.5)  # 150,150 셀
        goal_x = 7.5
        goal_y = 7.5

        # 목표점 장애물 겹침 확인 및 보정
        goal_candidate = self.find_free_goal_near(goal_x, goal_y)
        if goal_candidate is None:
            self.get_logger().error(f"Failed to find free goal near ({goal_x}, {goal_y})")
            self.goal = (0.0, 0.0)  # fallback
        else:
            self.goal = goal_candidate

        # 최종 경로 저장 변수
        self.planned_path_poses = []

        # FMS 경로 1회 발행 여부 플래그
        self.fms_path_published = False

        # matplotlib 설정
        plt.ion()
        self.fig, self.ax = plt.subplots()

        # 타이머 1초마다 콜백 실행
        self.timer = self.create_timer(1.0, self.timer_callback)

    def is_free_cell(self, x_cell, y_cell):
        if x_cell < 0 or x_cell >= self.width or y_cell < 0 or y_cell >= self.height:
            return False
        return self.map_data[y_cell, x_cell] == 0  # 0: 자유공간, 100: 장애물

    def find_free_goal_near(self, x_m, y_m, max_search=30):
        x_cell = int(x_m / self.resolution)
        y_cell = int(y_m / self.resolution)

        if self.is_free_cell(x_cell, y_cell):
            return (x_m, y_m)

        for radius in range(1, max_search + 1):
            for dx in range(-radius, radius + 1):
                for dy in range(-radius, radius + 1):
                    nx, ny = x_cell + dx, y_cell + dy
                    if self.is_free_cell(nx, ny):
                        return (nx * self.resolution, ny * self.resolution)
        return None

    def timer_callback(self):
        # map 발행 (매번)
        self.map_msg.header.stamp = self.get_clock().now().to_msg()
        self.map_msg.data = self.map_data.flatten().tolist()
        self.map_pub.publish(self.map_msg)

        # fms_planned_path 1회만 발행
        if not self.fms_path_published:
            self.get_logger().info("Publishing fms planned path for the first time")
            path_msg = Path()
            path_msg.header.frame_id = "map"
            path_msg.header.stamp = self.get_clock().now().to_msg()

            # 오직 goal 좌표 하나만 포함
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = self.goal[0]
            pose.pose.position.y = self.goal[1]
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

            self.fms_path_pub.publish(path_msg)
            self.fms_path_published = True

        # 그래프 그리기
        self.ax.clear()
        self.ax.set_title("Map, FMS Path and Planned Path")
        self.ax.set_xlabel("X (cells)")
        self.ax.set_ylabel("Y (cells)")

        self.ax.imshow(self.map_data, cmap='gray_r', origin='lower')

        start_cell = (int(self.start[0] / self.resolution), int(self.start[1] / self.resolution))
        goal_cell = (int(self.goal[0] / self.resolution), int(self.goal[1] / self.resolution))
        self.ax.plot(start_cell[0], start_cell[1], 'go', label='Start')
        self.ax.plot(goal_cell[0], goal_cell[1], 'ro', label='Goal')

        # fms_planned_path (직선) 그리기
        if self.fms_path_published:
            fms_x = [int(x / self.resolution) for x in np.linspace(self.start[0], self.goal[0], 30)]
            fms_y = [int(y / self.resolution) for y in np.linspace(self.start[1], self.goal[1], 30)]
            self.ax.plot(fms_x, fms_y, 'b-', label='FMS Planned Path')

        # planned_path (C++에서 받아온 최종 경로) 표시
        if self.planned_path_poses:
            planned_x = [int(p.x / self.resolution) for p in self.planned_path_poses]
            planned_y = [int(p.y / self.resolution) for p in self.planned_path_poses]
            self.ax.plot(planned_x, planned_y, 'y-', linewidth=2, label='Planned Path (A*)')

        self.ax.legend()
        plt.pause(0.1)

    def planned_path_callback(self, msg: Path):
        self.get_logger().info("Received planned_path")
        self.planned_path_poses = [pose.pose.position for pose in msg.poses]

        for i, p in enumerate(self.planned_path_poses):
            self.get_logger().info(f"Planned Path Point {i}: x={p.x:.3f}, y={p.y:.3f}")

def main(args=None):
    rclpy.init(args=args)
    node = DummyMapPathAndPlannedPathNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
