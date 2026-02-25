from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='evaluation',
            executable='gap_finder_base',
        ),
        Node(
            package='evaluation',
            executable='evaluation',
        )
        ]) 
