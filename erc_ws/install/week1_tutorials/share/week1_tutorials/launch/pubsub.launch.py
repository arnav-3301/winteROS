from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='week1_tutorials',
            executable='publisher',
        ),
        Node(
            package='week1_tutorials',
            executable='subscriber',
        ),
    ])