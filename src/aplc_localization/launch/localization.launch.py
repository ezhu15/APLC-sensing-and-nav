"""Start the APLC EKF (robot_localization ekf_node) with config/ekf.yaml."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use /clock from simulation. Set false on hardware.',
        ),
        DeclareLaunchArgument(
            'ekf_config',
            default_value=PathJoinSubstitution([FindPackageShare('aplc_localization'), 'config', 'ekf.yaml']),
            description='robot_localization parameter file.',
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            parameters=[
                LaunchConfiguration('ekf_config'),
                {'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)},
            ],
            output='screen',
        ),
    ])
