import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    # Declare launch description
    launch_description = LaunchDescription()

    # Get the path to the package's share directory
    share_dir = get_package_share_directory('sick_scan_xd')
    # Define the parameter file path (lidar_params.yaml)
    param_file_name = 'lrbot2_single_scan_params.yaml'
    param_file_path = os.path.join(share_dir, 'config', param_file_name)
    print("========================================================")
    print(param_file_path)
    print("========================================================")
    # # Add DeclareLaunchArgument
    # launch_description.add_action(
    #     DeclareLaunchArgument(
    #         'params_file',
    #         default_value=param_file_path,
    #         description='Path to the configuration file'
    #     )
    # )

    # Add DeclareLaunchArgument for use_sim_time
    launch_description.add_action(
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time if false'
        )
    )

    # Define Node
    lidar_node = Node(
        package='sick_scan_xd',
        executable='sick_generic_caller',
        name='sick_tim_571',
        output='screen',
        # parameters=[LaunchConfiguration('params_file')],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}, param_file_path],
        # parameters=[param_file_path],
        remappings=[
            ('/sick_tim_5xx/scan', 'scan_front'),
            ('cloud', 'cloud_front')
        ]
    )

    # Add Node to launch description
    launch_description.add_action(lidar_node)

    return launch_description
