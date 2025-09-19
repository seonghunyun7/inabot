from launch import LaunchDescription
from launch_ros.actions import LifecycleNode, Node

def generate_launch_description():
    return LaunchDescription([
        # 첫 번째 센서
        LifecycleNode(
            package="sick_safetyscanners2",
            executable="sick_safetyscanners2_lifecycle_node",
            name="sick_safetyscanners2_lifecycle_node_1",
            namespace="lidar1",
            output="screen",
            emulate_tty=True,
            parameters=[{
                "frame_id": "scan_1",
                "sensor_ip": "192.168.1.11",
                "host_ip": "192.168.1.9",
                "interface_ip": "0.0.0.0",
                "host_udp_port": 0
            }]
        ),

        # 두 번째 센서
        LifecycleNode(
            package="sick_safetyscanners2",
            executable="sick_safetyscanners2_lifecycle_node",
            name="sick_safetyscanners2_lifecycle_node_2",
            namespace="lidar2",
            output="screen",
            emulate_tty=True,
            parameters=[{
                "frame_id": "scan_2",
                "sensor_ip": "192.168.1.12",
                "host_ip": "192.168.1.9",
                "interface_ip": "0.0.0.0",
                "host_udp_port": 0
            }]
        ),

        # Lifecycle Manager (자동 활성화)
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_lidars",
            output="screen",
            parameters=[{
                "autostart": True,
                "node_names": [
                    "lidar1/sick_safetyscanners2_lifecycle_node_1",
                    "lidar2/sick_safetyscanners2_lifecycle_node_2"
                ]
            }]
        )
    ])
