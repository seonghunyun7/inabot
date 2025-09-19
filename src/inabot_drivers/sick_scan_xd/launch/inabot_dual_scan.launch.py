import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    launch_description = LaunchDescription()

    # Get the path to the package's share directory
    share_dir = get_package_share_directory('sick_scan_xd')
    print("1. ========================================================")
    print(share_dir)
    print("========================================================")
    # Define the parameter file path (lidar_params.yaml)
    param_file_name = 'lrbot2_dual_scan_params.yaml'
    param_file_path = os.path.join(share_dir, 'config', param_file_name)

    print("2. ========================================================")
    print(param_file_path)
    print("========================================================")

   # Node for sick_tim_5xx_front
    lidar_node_front = Node(
        package='sick_scan_xd',
        executable='sick_generic_caller',
        name='sick_tim_571_front',
        output='screen',
        parameters=[param_file_path],
        remappings=[
            ('/sick_tim_5xx/scan', 'scan_front'),
            ('cloud', 'cloud_front')
        ]
    )

    # Node for sick_tim_5xx_rear
    lidar_node_rear = Node(
        package='sick_scan_xd',
        executable='sick_generic_caller',
        name='sick_tim_571_rear',
        output='screen',
        parameters=[param_file_path],
        remappings=[
            ('/sick_tim_5xx/scan', 'scan_rear'),
            ('cloud', 'cloud_rear')
        ]
    )

    # Add Nodes to the launch description
    launch_description.add_action(lidar_node_front)
    launch_description.add_action(lidar_node_rear)

    return launch_description
