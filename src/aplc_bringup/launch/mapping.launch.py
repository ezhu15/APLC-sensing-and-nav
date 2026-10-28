"""Build an RTAB-Map map of the house world while driving the APLC.

Starts the simulation (Gazebo, robot, controllers, EKF) from aplc_gazebo/world.launch.py,
then rgbd_sync and RTAB-Map in SLAM mode, the map alignment recorder and RViz.

TF: the EKF publishes odom -> base_footprint (unchanged); RTAB-Map publishes
map -> odom and reads the robot's motion from the EKF's odom -> base_footprint.
Ground truth is never an input to RTAB-Map or the EKF; map_alignment reads it only to
record how the map frame sits in Gazebo's world, for evaluation.

The database is wiped at the start of every run.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# With colcon build --symlink-install this file is a symlink into src/aplc_bringup, so
# the database and the alignment file land in the source tree (maps/ is gitignored).
PKG_DIR = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
DEFAULT_DB = os.path.join(PKG_DIR, 'maps', 'rtabmap.db')
ALIGNMENT_FILE = os.path.join(PKG_DIR, 'config', 'map_alignment.yaml')

CAMERA = '/camera/camera/'


def generate_launch_description():
    os.makedirs(os.path.dirname(DEFAULT_DB), exist_ok=True)
    use_sim_time = {'use_sim_time': True}
    rtabmap_config = PathJoinSubstitution([FindPackageShare('aplc_bringup'), 'config', 'rtabmap.yaml'])
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true', description='Open the Gazebo GUI (drive with its Drive APLC panel).'),
        DeclareLaunchArgument('rviz', default_value='true', description='Open RViz with the growing map.'),
        DeclareLaunchArgument('rtabmap_viz', default_value='false',
                              description="Open RTAB-Map's own viewer (features, loop closures)."),
        DeclareLaunchArgument('database_path', default_value=DEFAULT_DB,
                              description='RTAB-Map database to create (wiped at start).'),
        DeclareLaunchArgument(
            'imu_gyro_bias', default_value='0.0017453',
            description='Simulated gyro bias (rad/s). Default 0.1 deg/s, the assumed residual after the '
                        "BNO085's onboard calibration (docs/localization/ekf.md).",
        ),
        DeclareLaunchArgument('seed', default_value='', description='Gazebo random seed (empty = random).'),

        # Simulation + robot + controllers + EKF (odom -> base_footprint). Spawns at the
        # default driveway start point. Its own RViz is replaced by the mapping view.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('aplc_gazebo'), 'launch', 'world.launch.py'])),
            launch_arguments={
                'gui': LaunchConfiguration('gui'),
                'rviz': 'false',
                'imu_gyro_bias': LaunchConfiguration('imu_gyro_bias'),
                'seed': LaunchConfiguration('seed'),
            }.items(),
        ),

        # Bundle color, depth and camera_info into one RGBDImage. The simulated camera
        # stamps all three identically, so exact sync works.
        Node(
            package='rtabmap_sync', executable='rgbd_sync', name='rgbd_sync', output='screen',
            parameters=[use_sim_time, {'approx_sync': False}],
            remappings=[
                ('rgb/image', CAMERA + 'color/image_raw'),
                ('depth/image', CAMERA + 'aligned_depth_to_color/image_raw'),
                ('rgb/camera_info', CAMERA + 'color/camera_info'),
            ],
        ),

        # RTAB-Map in SLAM mode. Motion comes from TF odom -> base_footprint (the EKF);
        # it publishes map -> odom, /map (2D grid) and its graph.
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
                'database_path': LaunchConfiguration('database_path'),
                'Mem/IncrementalMemory': 'true',       # SLAM: add new nodes
                'Mem/InitWMWithAllNodes': 'false',
            }],
            arguments=['--delete_db_on_start'],
        ),

        # Gazebo world -> map transform at mapping start, for comparing ground truth with
        # the estimate later (evaluation only).
        Node(
            package='aplc_bringup', executable='map_alignment', name='map_alignment', output='screen',
            parameters=[{'output': ALIGNMENT_FILE}],
        ),

        Node(
            package='rviz2', executable='rviz2', name='rviz2', output='screen',
            arguments=['-d', PathJoinSubstitution([FindPackageShare('aplc_bringup'), 'rviz', 'mapping.rviz'])],
            parameters=[use_sim_time],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
        Node(
            package='rtabmap_viz', executable='rtabmap_viz', name='rtabmap_viz', output='screen',
            parameters=[use_sim_time, {'frame_id': 'base_footprint', 'odom_frame_id': 'odom',
                                       'subscribe_rgbd': True, 'subscribe_depth': False}],
            condition=IfCondition(LaunchConfiguration('rtabmap_viz')),
        ),
    ])
