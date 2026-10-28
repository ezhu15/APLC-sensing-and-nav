"""Localize the APLC on a saved RTAB-Map map.

Starts the simulation (Gazebo, robot, controllers, EKF) from aplc_gazebo/world.launch.py,
then rgbd_sync and RTAB-Map in localization mode on the database from a mapping run.

Localization mode: the database is loaded, not wiped; no new map nodes are added; every
saved node is loaded into working memory so any part of the map can be recognized.

TF: the EKF publishes odom -> base_footprint (unchanged); RTAB-Map publishes map -> odom.
Before the first match with the map, map -> odom stays at its start value (identity,
because RGBD/StartAtOrigin assumes the robot starts where mapping started, the default
driveway spawn). Each match (about once per second when the camera sees mapped features)
corrects map -> odom.

Needs the database and its map_alignment.yaml from mapping.launch.py; stops with a
"run mapping first" message if either is missing.
"""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, LogInfo, OpaqueFunction, Shutdown,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# With colcon build --symlink-install this file is a symlink into src/aplc_bringup, so
# the default database is the one mapping.launch.py wrote in the source tree.
PKG_DIR = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
DEFAULT_DB = os.path.join(PKG_DIR, 'maps', 'rtabmap.db')
ALIGNMENT_NAME = 'map_alignment.yaml'

CAMERA = '/camera/camera/'


def map_files(context):
    """The database and its alignment file, and which of them are missing."""
    db = os.path.abspath(os.path.expanduser(LaunchConfiguration('database_path').perform(context)))
    alignment = os.path.join(os.path.dirname(db), ALIGNMENT_NAME)
    return db, alignment, [p for p in (db, alignment) if not os.path.isfile(p)]


def launch_setup(context):
    db, alignment, missing = map_files(context)
    if missing:
        # Nothing else is started: stop with a clear message.
        return [
            LogInfo(msg='Missing: ' + ', '.join(missing)),
            LogInfo(msg='Run mapping first: ros2 launch aplc_bringup mapping.launch.py '
                        '(see src/aplc_bringup/README.md).'),
            Shutdown(reason='map files missing'),
        ]
    use_sim_time = {'use_sim_time': True}
    rtabmap_config = PathJoinSubstitution([FindPackageShare('aplc_bringup'), 'config', 'rtabmap.yaml'])
    return [
        LogInfo(msg=f'Using map {db} with {alignment}'),

        # Simulation + robot + controllers + EKF (odom -> base_footprint), spawning at the
        # default driveway start point, where the map starts. Scoped so its arguments don't
        # leak into this launch file.
        GroupAction(scoped=True, actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('aplc_gazebo'), 'launch', 'world.launch.py'])),
            launch_arguments={
                'gui': LaunchConfiguration('gui'),
                'rviz': 'false',
                'imu_gyro_bias': LaunchConfiguration('imu_gyro_bias'),
                'seed': LaunchConfiguration('seed'),
            }.items(),
        )]),

        Node(
            package='rtabmap_sync', executable='rgbd_sync', name='rgbd_sync', output='screen',
            parameters=[use_sim_time, {'approx_sync': False}],
            remappings=[
                ('rgb/image', CAMERA + 'color/image_raw'),
                ('depth/image', CAMERA + 'aligned_depth_to_color/image_raw'),
                ('rgb/camera_info', CAMERA + 'color/camera_info'),
            ],
        ),

        # RTAB-Map in localization mode on the saved database (no --delete_db_on_start).
        Node(
            package='rtabmap_slam', executable='rtabmap', name='rtabmap', output='screen',
            parameters=[rtabmap_config, use_sim_time, {
                'frame_id': 'base_footprint',
                'odom_frame_id': 'odom',
                'map_frame_id': 'map',
                'publish_tf': True,
                'subscribe_rgbd': True,
                'subscribe_depth': False,
                'wait_for_transform': 0.2,
                'database_path': db,
                'Mem/IncrementalMemory': 'false',     # localization: no new map nodes
                'Mem/InitWMWithAllNodes': 'true',     # load every saved node into memory
                'RGBD/StartAtOrigin': 'true',         # robot starts where mapping started
            }],
        ),

        Node(
            package='rviz2', executable='rviz2', name='rviz2', output='screen',
            arguments=['-d', PathJoinSubstitution([FindPackageShare('aplc_bringup'), 'rviz', 'mapping.rviz'])],
            parameters=[use_sim_time],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true', description='Open the Gazebo GUI.'),
        DeclareLaunchArgument('rviz', default_value='true', description='Open RViz with the map.'),
        DeclareLaunchArgument('database_path', default_value=DEFAULT_DB,
                              description='RTAB-Map database from a mapping run; map_alignment.yaml must be '
                                          'in the same folder.'),
        DeclareLaunchArgument(
            'imu_gyro_bias', default_value='0.0',
            description='Simulated gyro bias (rad/s). Default 0, as in mapping (startup calibration assumed).',
        ),
        DeclareLaunchArgument('seed', default_value='', description='Gazebo random seed (empty = random).'),
        OpaqueFunction(function=launch_setup),
    ])
