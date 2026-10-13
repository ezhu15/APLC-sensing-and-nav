"""Launch Gazebo's server and GUI with an installed APLC world."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    world = PathJoinSubstitution([
        FindPackageShare('aplc_gazebo'), 'worlds', LaunchConfiguration('world')
    ])
    return LaunchDescription([
        DeclareLaunchArgument(
            'world', default_value='house_1.sdf',
            choices=['house_1.sdf', 'house_ramp.sdf'],
            description='Installed APLC world to load.',
        ),
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', '-v', '3', world],
            output='screen',
        ),
    ])
