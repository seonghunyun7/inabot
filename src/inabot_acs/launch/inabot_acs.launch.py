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
                {'broker_host': 'localhost',  # 브로커 호스트  # localhost # 192.168.0.10  as the broker address
                'broker_port': 1883,         # 브로커 포트
                'client_id': 'fms_client',
                'clean_session': True,       # 클린 세션 여부
                'keep_alive_interval': 60, # Keep-alive 간격 (초)
                'max_inflight': 65535,       # 최대 inflight 메시지 수
                'tls_enabled': False}        # TLS(SSL) 연결 사용 여부
                ],
            remappings=[
                # 예: ('/input_topic', '/robot/input_topic')
            ]
        )
    ])