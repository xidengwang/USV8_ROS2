from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    package_share = Path(get_package_share_directory('traj'))
    config_file = str(package_share / 'config' / 'traj.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'trajectory_source',
            default_value='fixed',
            description="Trajectory source: 'fixed' or 'csv'",
        ),
        Node(
            package='traj',
            executable='trajgen.py',
            name='trajgen',
            output='screen',
            parameters=[
                config_file,
                {'trajectory_source': LaunchConfiguration('trajectory_source')},
            ],
        ),
    ])
