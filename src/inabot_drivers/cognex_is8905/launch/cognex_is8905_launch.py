from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 첫 번째 카메라 (바닥/상단)
        Node(
            package='cognex_is8905',
            executable='cognex_is8905_node',
            name='cognex_is8905_camera1',
            output='screen',
            emulate_tty=True,
            parameters=[{'ip_address': '127.0.0.1', 'port': 5000}]
            #parameters=[{
            #    'ip_address': '192.168.1.50',
            #    'port': 5000
            #}]
        ),
    ])
