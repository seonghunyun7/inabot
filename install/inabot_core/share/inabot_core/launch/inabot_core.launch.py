from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    inabot_core_dir = get_package_share_directory('inabot_core')
    inabot_description_dir = get_package_share_directory('inabot_description')

    config_file = os.path.join(inabot_core_dir, 'config', 'params.yaml')

    urdf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(inabot_description_dir, 'launch', 'inabot_urdf_start.launch.py')
        )
    )

    inabot_node = Node(
        package='inabot_core',
        executable='inabot_core',
        output='screen',
        parameters=[config_file],
    )

    return LaunchDescription([
        urdf_launch,
        inabot_node,
    ])
