"""Publish APLC link transforms, optionally with stationary preview joints."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    model = PathJoinSubstitution([
        FindPackageShare('aplc_description'), 'urdf', 'aplc.urdf.xacro',
    ])
    robot_description = ParameterValue(
        Command([FindExecutable(name='xacro'), ' "', model, '"']),
        value_type=str,
    )
    use_sim_time = ParameterValue(
        LaunchConfiguration('use_sim_time'), value_type=bool,
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use /clock from simulation instead of wall time.',
        ),
        DeclareLaunchArgument(
            'publish_joint_states', default_value='true',
            description='Publish stationary preview joints; disable for Gazebo or hardware feedback.',
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': use_sim_time,
            }],
            output='screen',
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            condition=IfCondition(LaunchConfiguration('publish_joint_states')),
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
        ),
    ])
