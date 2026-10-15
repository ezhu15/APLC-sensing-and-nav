"""Open RViz with the APLC simulation view: robot model, TF, depth point cloud and color image."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'rviz_config',
            default_value=PathJoinSubstitution([FindPackageShare('aplc_gazebo'), 'rviz', 'aplc_sim.rviz']),
            description='RViz configuration file.',
        ),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use /clock from simulation; needed to match the simulated sensor timestamps.',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', LaunchConfiguration('rviz_config')],
            parameters=[{
                'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
            }],
            output='screen',
        ),
    ])
