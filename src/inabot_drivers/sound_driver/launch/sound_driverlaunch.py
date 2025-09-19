from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    
    return LaunchDescription([
        Node(
            package='sound_driver',
            executable='sound_node',
            name='sound_node',
            output='screen',
            parameters=[{
                'volume': 0.5,
            }]
        )
    ])
