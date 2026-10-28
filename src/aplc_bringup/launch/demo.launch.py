"""APLC demo: drive in Gazebo while the robot localizes on a saved RTAB-Map map.

One command starts Gazebo with the robot, the controllers and the EKF, RTAB-Map in
localization mode (localization.launch.py), the localization error monitor and RViz
(rviz/demo.rviz). Drive with Gazebo's Drive APLC panel or teleop in a second terminal.

Needs the database and its map_alignment.yaml from mapping.launch.py; stops with a
"run mapping first" message, starting nothing else, if either is missing.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, LogInfo, OpaqueFunction, Shutdown
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# With colcon build --symlink-install this file is a symlink into src/aplc_bringup.
PKG_DIR = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
DEFAULT_DB = os.path.join(PKG_DIR, 'maps', 'rtabmap.db')
ALIGNMENT_NAME = 'map_alignment.yaml'
CSV_DIR = os.path.join(PKG_DIR, 'logs')   # gitignored


def launch_setup(context):
    db = os.path.abspath(os.path.expanduser(LaunchConfiguration('database_path').perform(context)))
    alignment = os.path.join(os.path.dirname(db), ALIGNMENT_NAME)
    missing = [p for p in (db, alignment) if not os.path.isfile(p)]
    if missing:
        return [
            LogInfo(msg='Missing: ' + ', '.join(missing)),
            LogInfo(msg='Run mapping first: ros2 launch aplc_bringup mapping.launch.py '
                        '(see src/aplc_bringup/README.md).'),
            Shutdown(reason='map files missing'),
        ]
    csv_dir = CSV_DIR if LaunchConfiguration('error_csv').perform(context).lower() == 'true' else ''
    return [
        # Gazebo + robot + EKF + rgbd_sync + RTAB-Map localization. Scoped so the
        # arguments passed in don't leak into this launch file; its own RViz is off.
        GroupAction(scoped=True, actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('aplc_bringup'), 'launch', 'localization.launch.py'])),
            launch_arguments={
                'gui': LaunchConfiguration('gui'),
                'rviz': 'false',
                'database_path': db,
                'imu_gyro_bias': LaunchConfiguration('imu_gyro_bias'),
            }.items(),
        )]),

        # Live error against ground truth (evaluation only), paths for RViz, correction count.
        Node(
            package='aplc_bringup', executable='localization_monitor', name='localization_monitor',
            output='screen',
            parameters=[{'alignment_file': alignment, 'csv_dir': csv_dir}],
        ),

        Node(
            package='rviz2', executable='rviz2', name='rviz2', output='screen',
            arguments=['-d', PathJoinSubstitution([FindPackageShare('aplc_bringup'), 'rviz', 'demo.rviz'])],
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true', description='Open the Gazebo GUI (drive with its Drive APLC panel).'),
        DeclareLaunchArgument('rviz', default_value='true', description='Open RViz with the demo view.'),
        DeclareLaunchArgument('database_path', default_value=DEFAULT_DB,
                              description='RTAB-Map database from a mapping run; map_alignment.yaml must be '
                                          'in the same folder.'),
        DeclareLaunchArgument(
            'imu_gyro_bias', default_value='0.0',
            description='Simulated gyro bias (rad/s). Default 0, as in mapping (startup calibration assumed).',
        ),
        DeclareLaunchArgument('error_csv', default_value='false',
                              description='Also log the error over time to src/aplc_bringup/logs/ (gitignored).'),
        OpaqueFunction(function=launch_setup),
    ])
