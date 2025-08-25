import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Bool
import math
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import threading


def quaternion_from_euler(roll, pitch, yaw):
    qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - \
         math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
    qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + \
         math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
    qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - \
         math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
    qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + \
         math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
    return [qx, qy, qz, qw]


class DummyOdomAndPathNode(Node):
    def __init__(self):
        super().__init__('dummy_odom_and_path_node')

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.linear_velocity = 0.6
        self.angular_velocity = 0.0
        self.last_time = self.get_clock().now()

        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.path_pub = self.create_publisher(Path, 'planned_path', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.goal_reached_sub = self.create_subscription(Bool, 'goal_reached', self.goal_reached_callback, 10)

        self.cmd_vel_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)

        self.path_msg = Path()
        self.path_msg.header.frame_id = 'odom'

        self.lock = threading.Lock()

        self.add_initial_goal()
        self.timer = self.create_timer(0.05, self.timer_callback)
        self.publish_initial_cmd_vel()

        self.get_logger().info('DummyOdomAndPathNode started')

    def publish_initial_cmd_vel(self):
        twist_msg = Twist()
        twist_msg.linear.x = self.linear_velocity
        twist_msg.angular.z = self.angular_velocity
        self.cmd_vel_pub.publish(twist_msg)
        self.get_logger().info(
            f"최초 cmd_vel 발행: linear.x={self.linear_velocity:.2f}, angular.z={self.angular_velocity:.2f}"
        )

    def cmd_vel_callback(self, msg):
        self.linear_velocity = msg.linear.x
        self.angular_velocity = msg.angular.z
        # Uncomment below to enable logging of cmd_vel updates
        # self.get_logger().info(
        #     f"cmd_vel 수신: linear.x={self.linear_velocity:.2f}, angular.z={self.angular_velocity:.2f}"
        # )

    def timer_callback(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now
        if dt <= 0:
            return

        self.x += self.linear_velocity * math.cos(self.yaw) * dt
        self.y += self.linear_velocity * math.sin(self.yaw) * dt
        self.yaw += self.angular_velocity * dt

        odom_msg = Odometry()
        odom_msg.header.stamp = now.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        q = quaternion_from_euler(0, 0, self.yaw)
        odom_msg.pose.pose.orientation.x = q[0]
        odom_msg.pose.pose.orientation.y = q[1]
        odom_msg.pose.pose.orientation.z = q[2]
        odom_msg.pose.pose.orientation.w = q[3]
        odom_msg.twist.twist.linear.x = self.linear_velocity
        odom_msg.twist.twist.angular.z = self.angular_velocity
        self.odom_pub.publish(odom_msg)

        with self.lock:
            self.path_msg.header.stamp = now.to_msg()
            self.path_pub.publish(self.path_msg)

    def generate_next_goal(self, distance=0.4):
        next_x = self.x + distance * math.cos(self.yaw)
        next_y = self.y + distance * math.sin(self.yaw)
        return (next_x, next_y)

    def add_initial_goal(self):
        init_goal = self.generate_next_goal()
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = 'odom'
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.pose.position.x = init_goal[0]
        pose_stamped.pose.position.y = init_goal[1]
        pose_stamped.pose.orientation.w = 1.0

        with self.lock:
            self.path_msg.poses.clear()
            self.path_msg.poses.append(pose_stamped)
            self.path_pub.publish(self.path_msg)

        self.get_logger().info(f"초기 목표점 생성: ({init_goal[0]:.2f}, {init_goal[1]:.2f})")

    def goal_reached_callback(self, msg):
        if msg.data:
            next_goal = self.generate_next_goal()
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = 'odom'
            pose_stamped.header.stamp = self.get_clock().now().to_msg()
            pose_stamped.pose.position.x = next_goal[0]
            pose_stamped.pose.position.y = next_goal[1]
            pose_stamped.pose.orientation.w = 1.0

            with self.lock:
                self.path_msg.poses.clear()
                self.path_msg.poses.append(pose_stamped)
                self.path_pub.publish(self.path_msg)

            self.get_logger().info(f"새 목표점 생성: ({next_goal[0]:.2f}, {next_goal[1]:.2f})")


def ros_spin_thread(node):
    rclpy.spin(node)


def main(args=None):
    rclpy.init(args=args)
    node = DummyOdomAndPathNode()

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 8))
    point_path_goal, = ax1.plot([], [], 'bs', label='Current Goal')  # 파란 사각형 점으로 목표점 표시
    point_odom, = ax1.plot([], [], 'ro', label='Current Position')
    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.set_title('Odometry and Current Goal Visualization')
    ax1.legend()
    ax1.grid(True)

    time_data = []
    linear_vel_data = []
    angular_vel_data = []
    line_linear, = ax2.plot([], [], 'g-', label='Linear Velocity (m/s)')
    line_angular, = ax2.plot([], [], 'r-', label='Angular Velocity (rad/s)')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Velocity')
    ax2.set_title('cmd_vel Linear and Angular Velocity')
    ax2.legend()
    ax2.grid(True)

    spin_thread = threading.Thread(target=ros_spin_thread, args=(node,), daemon=True)
    spin_thread.start()

    start_time = node.get_clock().now().nanoseconds * 1e-9

    def update(frame):
        now = node.get_clock().now().nanoseconds * 1e-9
        elapsed = now - start_time

        with node.lock:
            if node.path_msg.poses:
                # path_msg 내 가장 최신(현재) 목표점 하나만 표시
                goal_pose = node.path_msg.poses[-1]
                goal_x = goal_pose.pose.position.x
                goal_y = goal_pose.pose.position.y
                point_path_goal.set_data(goal_x, goal_y)
            else:
                point_path_goal.set_data([], [])

            point_odom.set_data(node.x, node.y)

            margin = 1.0
            all_x = [node.x]
            all_y = [node.y]
            if node.path_msg.poses:
                all_x.append(goal_x)
                all_y.append(goal_y)

            ax1.set_xlim(min(all_x) - margin, max(all_x) + margin)
            ax1.set_ylim(min(all_y) - margin, max(all_y) + margin)

        time_data.append(elapsed)
        linear_vel_data.append(node.linear_velocity)
        angular_vel_data.append(node.angular_velocity)

        line_linear.set_data(time_data, linear_vel_data)
        line_angular.set_data(time_data, angular_vel_data)
        ax2.set_xlim(max(0, elapsed - 10), elapsed + 1)

        all_vel = linear_vel_data + angular_vel_data
        ax2.set_ylim(min(all_vel) - 0.1, max(all_vel) + 0.1)

        return point_path_goal, point_odom, line_linear, line_angular

    ani = FuncAnimation(fig, update, interval=50, blit=False)
    plt.show()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
