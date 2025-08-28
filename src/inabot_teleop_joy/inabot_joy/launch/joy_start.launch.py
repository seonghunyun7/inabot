from launch_ros.actions import Node
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution

def generate_launch_description():

    return LaunchDescription([

        # device_id:=0 → /dev/input/js0 (첫 번째 조이스틱)
        ExecuteProcess(
            cmd=['ros2', 'run', 'joy', 'joy_node', '--ros-args', '-p', 'device_id:=0'],
            output='screen'
        ),

        ExecuteProcess(
            cmd=['ros2', 'run', 'inabot_joy', 'joy_con'],
            output='screen'
        ),
    ])
