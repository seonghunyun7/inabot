from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='inabot_ota',
            executable='ota_node',
            name='ota_node',
            output='screen',
            parameters=[{
                'INSTALL_DIR_': '/home/ysh/inabot_ws/install',
                'backup_dir_base': '/home/ysh/inabot_ws/install_backup_',
                'temp_archive': '/home/ysh/tmp/ota_update.tar.gz',  # <- 변경
                'ota_port': 5005,                       # 포트 번호
                'ota_host': '127.0.0.1'                   # 수신 주소 (기본은 모든 인터페이스)
            }]
        )
    ])
