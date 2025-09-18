from launch import LaunchDescription
from launch_ros.actions import Node
from datetime import datetime

def generate_launch_description():
    # -----------------------------
    # 로봇 식별 정보
    # -----------------------------
    manufacturer = 'RobotCompany'   # 각 제조사별로 변경 가능
    serial_number = 'Robot001'      # 각 로봇별로 변경 가능
    vda_interface = 'uagv'
    vda_version = 'v2.0.0'
    vda_full_version = '2.0.0'

    # -----------------------------
    # 토픽 및 LWT payload 구성
    # -----------------------------
    order_topic = f"{vda_interface}/{vda_version}/{manufacturer}/{serial_number}/order"
    lwt_topic = f"{vda_interface}/{vda_version}/{manufacturer}/{serial_number}/connection"
    
    # 현재 시간 기준 ISO 8601 timestamp
    timestamp = datetime.utcnow().strftime("%Y-%m-%dT%H:%M:%SZ")
    
    lwt_payload = (
        f'{{"headerId":0,"timestamp":"{timestamp}",'
        f'"version":"{vda_full_version}",'
        f'"manufacturer":"{manufacturer}",'
        f'"serialNumber":"{serial_number}",'
        f'"connectionState":"CONNECTIONBROKEN"}}'
    )

    # -----------------------------
    # Node 구성
    # -----------------------------
    return LaunchDescription([
        Node(
            package='inabot_acs',
            executable='robot_acs_node',
            name='robot_acs_node',
            output='screen',
            parameters=[
                {
                    # MQTT 브로커 설정
                    'broker_host': 'localhost',  # 실제 브로커 IP
                    'broker_port': 1883,
                    'client_id': 'fms_client',
                    'clean_session': True,
                    'keep_alive_interval': 15,
                    'max_inflight': 65535,
                    'tls_enabled': False,  # 운영에서는 TLS 권장
                    #'username': "",
                    #'password': "",

                    # 로봇 식별
                    'manufacturer': manufacturer,
                    'serial_number': serial_number,
                    'vda_interface': vda_interface,
                    'vda_version': vda_version,
                    'vda_full_version': vda_full_version,

                    # Heartbeat 설정
                    'heartbeat_interval': 5,

                    # LWT 설정
                    'lwt_enabled': True,
                    'lwt_topic': lwt_topic,
                    'lwt_payload': lwt_payload,
                    'lwt_qos': 1,
                    'lwt_retain': True,

                    # VDA5050 Order topic
                    #'order_topic': order_topic
                }
            ]
        )
    ])
