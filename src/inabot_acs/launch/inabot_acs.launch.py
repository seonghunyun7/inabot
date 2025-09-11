from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='inabot_acs',
            executable='robot_acs_node',
            name='robot_acs_node',
            output='screen',
            parameters=[
                {
                    'broker_host': 'localhost',
                    'broker_port': 1883,
                    'client_id': 'fms_client',
                    'clean_session': True,
                    'keep_alive_interval': 15,
                    'max_inflight': 65535,
                    'tls_enabled': False,
                    'manufacturer': 'Inatech',      # 임시 값
                    'serial_number': 'P3LDD02'      # 임시 값
                }
            ],
            remappings=[
                # 예: ('/input_topic', '/robot/input_topic')
            ]
        )
    ])