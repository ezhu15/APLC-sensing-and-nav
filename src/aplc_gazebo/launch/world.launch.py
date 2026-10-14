"""Start an APLC world with GUI driving and spawn the robot."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    world = PathJoinSubstitution([
        FindPackageShare('aplc_gazebo'), 'worlds', LaunchConfiguration('world')
    ])
    gui_config = PathJoinSubstitution([
        FindPackageShare('aplc_gazebo'), 'config', 'gui.config'
    ])
    # Both installed SDF files name their world "house_ramp". The create
    # utility waits for the description and the world's creation service.
    spawn_robot = Node(
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
    )
    # The controller manager runs inside Gazebo, so it exists only once the robot is spawned.
    # Physics starts automatically so controller activation can complete.
    spawn_controllers = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster', 'diff_drive_controller',
            '--switch-timeout', '600', '--service-call-timeout', '610',
        ],
        output='screen',
    )
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
            cmd=['gz', 'sim', '-r', '-v', '3', '--gui-config', gui_config, world],
            output='screen',
        ),
        # Gazebo's simulation clock for nodes with use_sim_time ('[' = Gazebo to ROS).
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                '/gui/cmd_vel@geometry_msgs/msg/Twist[gz.msgs.Twist',
            ],
            output='screen',
        ),
        Node(
            package='aplc_gazebo',
            executable='stamp_cmd_vel.py',
            parameters=[{'use_sim_time': True}],
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
        spawn_robot,
        RegisterEventHandler(OnProcessExit(
            target_action=spawn_robot, on_exit=[spawn_controllers],
        )),
    ])
