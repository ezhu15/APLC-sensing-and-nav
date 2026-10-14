"""Start the APLC world, spawn the parked car and the robot, start the wheel controllers,
bridge clock and camera topics to ROS, and build the camera point cloud."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    world = PathJoinSubstitution([
        FindPackageShare('aplc_gazebo'), 'worlds', LaunchConfiguration('world')
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
    # Activation needs the simulation running: with paused:=true, allow 10 minutes to press Play.
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
        DeclareLaunchArgument('x', default_value='5.0', description='Robot spawn x position (m).'),
        DeclareLaunchArgument('y', default_value='-3.0', description='Robot spawn y position (m).'),
        DeclareLaunchArgument(
            'z', default_value='0.25',
            description='Robot spawn height of base_link (m); it rests 0.203 m above the ground.',
        ),
        DeclareLaunchArgument(
            'yaw', default_value='3.14159',
            description='Robot spawn yaw (rad); the default faces the house, away from the parked car.',
        ),
        DeclareLaunchArgument('car', default_value='true', description='Spawn the parked RAV4.'),
        DeclareLaunchArgument(
            'car_x', default_value='7.0',
            description='Car rear bumper center x (m). The driveway spans x 2 to 14.',
        ),
        DeclareLaunchArgument(
            'car_y', default_value='-3.4',
            description='Car rear bumper center y (m). The driveway is centered at y -3.4, 3.5 m wide.',
        ),
        DeclareLaunchArgument(
            'car_yaw', default_value='0.0',
            description='Car heading (rad); 0 points the front at the street, the trunk at the house.',
        ),
        DeclareLaunchArgument('camera', default_value='true', description='Add the RGB-D camera.'),
        DeclareLaunchArgument(
            'camera_model', default_value='d435', choices=['d435', 'd435i', 'd455'],
            description='RealSense model.',
        ),
        DeclareLaunchArgument(
            'camera_xyz', default_value='0.2062 0 0.02',
            description='Camera bottom screw position in base_link (m); default: chassis front face.',
        ),
        DeclareLaunchArgument(
            'camera_rpy', default_value='0 0 0',
            description='Camera roll pitch yaw in base_link (rad); positive pitch tilts it down.',
        ),
        DeclareLaunchArgument(
            'render_engine', default_value='ogre', choices=['ogre', 'ogre2'],
            description='ogre works in VMs; ogre2 is needed for camera noise.',
        ),
        DeclareLaunchArgument(
            'paused', default_value='false',
            description='Start Gazebo paused; controllers then activate only after pressing Play.',
        ),
        # '-r' runs the simulation immediately. Spawning works either way.
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', '-v', '3', '--render-engine', LaunchConfiguration('render_engine'), world],
            condition=UnlessCondition(LaunchConfiguration('paused')),
            output='screen',
        ),
        ExecuteProcess(
            cmd=['gz', 'sim', '-v', '3', '--render-engine', LaunchConfiguration('render_engine'), world],
            condition=IfCondition(LaunchConfiguration('paused')),
            output='screen',
        ),
        # Clock and camera topics; see config/bridge.yaml.
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            parameters=[{
                'config_file': PathJoinSubstitution([
                    FindPackageShare('aplc_gazebo'), 'config', 'bridge.yaml',
                ]),
                'use_sim_time': True,
            }],
            output='screen',
        ),
        # Colored point cloud from the aligned depth and color images, in the optical
        # frame, like realsense2_camera's /camera/camera/depth/color/points.
        ComposableNodeContainer(
            name='camera_processing',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            condition=IfCondition(LaunchConfiguration('camera')),
            composable_node_descriptions=[ComposableNode(
                package='depth_image_proc',
                plugin='depth_image_proc::PointCloudXyzrgbNode',
                name='point_cloud_xyzrgb',
                remappings=[
                    ('rgb/image_rect_color', '/camera/camera/color/image_raw'),
                    ('rgb/camera_info', '/camera/camera/color/camera_info'),
                    ('depth_registered/image_rect', '/camera/camera/aligned_depth_to_color/image_raw'),
                    ('depth_registered/points', '/camera/camera/depth/color/points'),
                ],
                parameters=[{'use_sim_time': True}],
            )],
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
                'camera': LaunchConfiguration('camera'),
                'camera_model': LaunchConfiguration('camera_model'),
                'camera_xyz': LaunchConfiguration('camera_xyz'),
                'camera_rpy': LaunchConfiguration('camera_rpy'),
            }.items(),
        ),
        Node(
            package='ros_gz_sim',
            executable='create',
            name='spawn_car',
            condition=IfCondition(LaunchConfiguration('car')),
            arguments=[
                '-world', 'house_ramp',
                '-name', 'car',
                '-file', PathJoinSubstitution([FindPackageShare('aplc_gazebo'), 'models', 'rav4.sdf']),
                '-x', LaunchConfiguration('car_x'),
                '-y', LaunchConfiguration('car_y'),
                '-z', '0',
                '-Y', LaunchConfiguration('car_yaw'),
            ],
            output='screen',
        ),
        spawn_robot,
        RegisterEventHandler(OnProcessExit(
            target_action=spawn_robot, on_exit=[spawn_controllers],
        )),
    ])
