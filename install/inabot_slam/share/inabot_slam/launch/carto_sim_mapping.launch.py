"""
  Copyright 2018 The Cartographer Authors
  Copyright 2022 Wyca Robotics (for the ros2 conversion)

  Licensed under the Apache License, Version 2.0 (the "License");
  you may not use this file except in compliance with the License.
  You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.
"""

from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import Shutdown
from launch.actions import TimerAction
import os

def generate_launch_description():
    ## ***** Launch arguments *****
    bag_filename_arg = DeclareLaunchArgument('bag_filename')

    carto_mapping = get_package_share_directory('inabot_slam')
    cartographer_config_dir = LaunchConfiguration('cartographer_config_dir', default=os.path.join(
                                                  carto_mapping, 'config'))
    configuration_basename = LaunchConfiguration('configuration_basename',
                                                 default='cartographer_2d_lidar.lua')

  ## ***** File paths ******
    pkg_share = FindPackageShare('cartographer_ros').find('cartographer_ros')
    urdf_dir = os.path.join(pkg_share, 'urdf')
    urdf_file = os.path.join(urdf_dir, 'backpack_2d.urdf')
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    ## ***** Nodes *****
    
    #robot_state_publisher_node = Node(
    #    package = 'robot_state_publisher',
    #    executable = 'robot_state_publisher',
    #    parameters=[
    #        #{'robot_description': robot_desc},
    #        {'use_sim_time': True}],
    #    output = 'screen'
    #    )

    cartographer_node = Node(
        package = 'cartographer_ros',
        executable = 'cartographer_node',
        parameters = [{'use_sim_time': True}],
        arguments = [
            '-configuration_directory', cartographer_config_dir,
            '-configuration_basename', configuration_basename],
        remappings=[
            #('scan', 'scan_merged'),
            ('scan', 'scan_front'),
            #('scan1', 'scan_rear'),
        ],
        output = 'screen'
        )

    cartographer_occupancy_grid_node = Node(
        package = 'cartographer_ros',
        executable = 'cartographer_occupancy_grid_node',
        parameters = [
            {'use_sim_time': True},
            {'resolution': 0.05}],
        )

    rviz_node = Node(
        package = 'rviz2',
        executable = 'rviz2',
        on_exit = Shutdown(),
        arguments = ['-d', FindPackageShare('cartographer_ros').find('cartographer_ros') + '/configuration_files/demo_2d.rviz'],
        parameters = [{'use_sim_time': True}],
    )

    # ros2 launch inabot_slam cartop_sim_mapping.launch.py bag_filename:=/home/yoon/hs_bag/hs_0313_bag_0.db3
    #ros2_bag_play_cmd = ExecuteProcess(
     #  cmd = ['ros2', 'bag', 'play', LaunchConfiguration('bag_filename'), '--clock'],
    #   name = 'rosbag_play',
    #)
    ros2_bag_play_cmd = TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(
                cmd = ['ros2', 'bag', 'play', LaunchConfiguration('bag_filename'), '--clock'],
                name = 'rosbag_play',
            )
        ]
    )

    return LaunchDescription([
        # Launch arguments
        bag_filename_arg,
        # Nodes
        #robot_state_publisher_node,
        cartographer_node,
        cartographer_occupancy_grid_node,
        rviz_node,
        ros2_bag_play_cmd
    ])
