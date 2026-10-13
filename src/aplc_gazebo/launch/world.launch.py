"""Start a paused APLC world and spawn the robot at its initial pose."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
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
        DeclareLaunchArgument('x', default_value='5.0', description='Spawn x position (m).'),
        DeclareLaunchArgument('y', default_value='-3.0', description='Spawn y position (m).'),
        DeclareLaunchArgument(
            'z', default_value='0.25',
            description='Spawn height of base_link (m); it rests 0.203 m above the ground.',
        ),
        DeclareLaunchArgument('yaw', default_value='0.0', description='Spawn yaw (rad).'),
        ExecuteProcess(
            cmd=['gz', 'sim', '-v', '3', world],
            output='screen',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('aplc_description'),
                'launch', 'description.launch.py',
            ])),
            launch_arguments={
                'publish_joint_states': 'false',
                'use_sim_time': 'true',
            }.items(),
        ),
        # Both installed SDF files name their world "house_ramp". The create
        # utility waits for the description and the world's creation service.
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-world', 'house_ramp',
                '-name', 'aplc',
                '-topic', '/robot_description',
                '-x', LaunchConfiguration('x'),
                '-y', LaunchConfiguration('y'),
                '-z', LaunchConfiguration('z'),
                '-R', '0', '-P', '0', '-Y', LaunchConfiguration('yaw'),
            ],
            output='screen',
        ),
    ])
