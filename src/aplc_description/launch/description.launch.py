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
        Command([
            FindExecutable(name='xacro'), ' "', model, '"',
            ' camera:=', LaunchConfiguration('camera'),
            ' camera_model:=', LaunchConfiguration('camera_model'),
            ' camera_xyz:="', LaunchConfiguration('camera_xyz'), '"',
            ' camera_rpy:="', LaunchConfiguration('camera_rpy'), '"',
            ' imu_xyz:="', LaunchConfiguration('imu_xyz'), '"',
            ' imu_rpy:="', LaunchConfiguration('imu_rpy'), '"',
            ' imu_gyro_bias:=', LaunchConfiguration('imu_gyro_bias'),
            ' imu_accel_bias:=', LaunchConfiguration('imu_accel_bias'),
        ]),
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
        DeclareLaunchArgument('camera', default_value='true', description='Add the RGB-D camera.'),
        DeclareLaunchArgument(
            'camera_model', default_value='d435', choices=['d435', 'd435i', 'd455'],
            description='RealSense model (realsense2_description).',
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
            'imu_xyz', default_value='0 0 0',
            description='IMU (imu_link) position in base_link (m); default: chassis center.',
        ),
        DeclareLaunchArgument(
            'imu_rpy', default_value='0 0 0', description='IMU roll pitch yaw in base_link (rad).',
        ),
        DeclareLaunchArgument(
            'imu_gyro_bias', default_value='0.017453',
            description='Gyro bias magnitude per axis (rad/s); BMF055 typ 1 deg/s. 0 = calibrated.',
        ),
        DeclareLaunchArgument(
            'imu_accel_bias', default_value='0.784532',
            description='Accel bias magnitude per axis (m/s^2); BMF055 typ 80 mg. 0 = calibrated.',
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
