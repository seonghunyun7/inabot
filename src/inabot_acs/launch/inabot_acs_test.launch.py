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
                    # MQTT 브로커 설정
                    'broker_host': 'localhost',      # 로컬 테스트 브로커
                    'broker_port': 1883,
                    'client_id': 'fms_client',
                    'clean_session': False,           # LWT 테스트용
                    'keep_alive_interval': 10,       # MQTT keepalive (초)
                    'max_inflight': 65535,
                    'tls_enabled': False,
                    
                    # 로봇 식별
                    'manufacturer': 'Inatech',
                    'serial_number': 'P3LDD02',

                    # Heartbeat 설정
                    'heartbeat_interval': 5,          # Heartbeat 주기 (초)
                    
                    # LWT 설정
                    'lwt_enabled': True,
                    'lwt_topic': 'uagv/v2/Inatech/P3LDD02/connection',
                    'lwt_payload': '{"headerId":0,"timestamp":"2025-09-11T00:00:00Z","version":"2.0.0","manufacturer":"Inatech","serialNumber":"P3LDD02","connectionState":"OFFLINE"}',
                    'lwt_qos': 1,
                    'lwt_retain': True
                }
            ]
        )
    ])
