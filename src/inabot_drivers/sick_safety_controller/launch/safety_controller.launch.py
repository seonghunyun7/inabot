from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sick_safety_controller',
            executable='safety_controller_node',
            output='screen',
            parameters=[{
                "modbus_ip": "127.0.0.1", # "192.168.0.10",
                "modbus_port": 5020 # 502
            }]
        )
    ])