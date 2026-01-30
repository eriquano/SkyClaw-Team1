from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='go_green',
            executable='go_green',
            name='go_green',
            output='screen'
        ),
        Node(
            package='go_green',
            executable='green_tracker',
            name='green_tracker',
            output='screen'
        )
    ])