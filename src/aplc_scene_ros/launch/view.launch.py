from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory('aplc_scene_ros'))
    return LaunchDescription([
        DeclareLaunchArgument('map',default_value=str(share/'maps/quest_20261104.scene.json')),
        DeclareLaunchArgument('rviz',default_value='true'),
        DeclareLaunchArgument('live',default_value='false'),
        DeclareLaunchArgument('pov',default_value='true'),
        DeclareLaunchArgument('bind_address',default_value='0.0.0.0'),
        DeclareLaunchArgument('pose_port',default_value='5005'),
        DeclareLaunchArgument('alignment_port',default_value='5006'),
        DeclareLaunchArgument('show_ceilings',default_value='false'),
        DeclareLaunchArgument('show_labels',default_value='true'),
        Node(package='aplc_scene_ros',executable='scene_publisher',output='screen',parameters=[{
            'map_file':LaunchConfiguration('map'),
            'show_ceilings':ParameterValue(LaunchConfiguration('show_ceilings'),value_type=bool),
            'show_labels':ParameterValue(LaunchConfiguration('show_labels'),value_type=bool)}]),
        Node(package='aplc_scene_ros',executable='quest_bridge',output='screen',
             condition=IfCondition(LaunchConfiguration('live')),parameters=[{
                 'map_file':LaunchConfiguration('map'),
                 'bind_address':LaunchConfiguration('bind_address'),
                 'pose_port':ParameterValue(LaunchConfiguration('pose_port'),value_type=int),
                 'alignment_port':ParameterValue(LaunchConfiguration('alignment_port'),value_type=int)}]),
        Node(package='rviz2',executable='rviz2',name='aplc_overview',arguments=['-d',str(share/'rviz/scene.rviz')],
             condition=IfCondition(LaunchConfiguration('rviz'))),
        Node(package='rviz2',executable='rviz2',name='aplc_quest_pov',arguments=['-d',str(share/'rviz/quest_pov.rviz')],
             condition=IfCondition(PythonExpression(["'",LaunchConfiguration('rviz'),"'.lower() == 'true' and '",LaunchConfiguration('pov'),"'.lower() == 'true'"]))),
    ])
