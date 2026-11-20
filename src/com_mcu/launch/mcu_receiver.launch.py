from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        Node(
            package='com_mcu',
            executable='uart_example_accept.py',
            name='uart_accept',
            output='screen',
        )
    ])
