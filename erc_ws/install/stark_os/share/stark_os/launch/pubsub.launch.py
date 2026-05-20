from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='stark_os',
            executable='publisher1',
        ),
        Node(
            package='stark_os',
            executable='publisher2',
        ),
        Node(
            package='stark_os',
            executable='subscriber',
        ),
    ])