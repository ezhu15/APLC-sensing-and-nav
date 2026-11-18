"""Route Server demo: the APLC drives a predefined route graph from the front door landing
to the car's trunk, avoiding obstacles seen by a headset-height depth camera, with the map
frame provided by a simulated Meta Quest.

Starts world.launch.py (Gazebo, robot, controllers, EKF) with the depth camera moved up to
the headset mount, then fake_quest (map -> odom), Nav2 (route_server, controller_server
with MPPI, planner_server for the global costmap, behavior_server, bt_navigator) and RViz.

  ros2 launch aplc_navigation route_demo.launch.py
  ros2 run aplc_navigation route_demo --goal trunk          # drive the route
  ros2 run aplc_navigation obstacles box | block | walker   # add obstacles
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# base_footprint -> base_link height (aplc.urdf.xacro: height / 2 + wheel_radius).
BASE_LINK_Z = 0.2032
NAV2_NODES = ['controller_server', 'planner_server', 'route_server', 'behavior_server', 'bt_navigator']


def setup(context):
    arg = lambda name: LaunchConfiguration(name).perform(context)
    share = lambda *p: PathJoinSubstitution([FindPackageShare('aplc_navigation'), *p]).perform(context)
    hx, hz, pitch = float(arg('headset_x')), float(arg('headset_height')), float(arg('headset_pitch'))
    params = share('config', 'nav2_route_demo.yaml')
    sim = {'use_sim_time': True}
    cmd_vel = [('cmd_vel', '/diff_drive_controller/cmd_vel')]

    world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('aplc_gazebo'), 'launch', 'world.launch.py'])),
        launch_arguments={
            'world': arg('world'), 'x': arg('x'), 'y': arg('y'), 'z': arg('z'), 'yaw': arg('yaw'),
            'gui': arg('gui'), 'rviz': 'false', 'seed': arg('seed'),
            'render_engine': arg('render_engine'),
            # The depth camera stands in for the headset's view: same mount, looking down.
            'camera_xyz': f'{hx} 0 {hz - BASE_LINK_Z:.4f}',
            'camera_rpy': f'0 {pitch} 0',
            'camera_width': arg('camera_width'), 'camera_height': arg('camera_height'),
            'camera_rate': arg('camera_rate'),
            'imu_gyro_bias': arg('imu_gyro_bias'),
            # /wheel_encoders is not used by the EKF or Nav2; save the CPU.
            'wheel_encoders': 'false',
        }.items(),
    )
    quest = Node(
        package='aplc_navigation', executable='fake_quest', name='fake_quest', output='screen',
        parameters=[sim, {
            'mount_xyz': [hx, 0.0, hz], 'mount_rpy': [0.0, 0.0, 0.0],
            'position_noise_std': float(arg('quest_noise')),
            'drift_xy_max': float(arg('quest_drift_max')),
            'dropout_period': float(arg('quest_dropout_period')),
            'seed': int(arg('seed') or 0),
        }],
    )
    nav2 = [
        Node(package='nav2_controller', executable='controller_server', output='screen',
             parameters=[params, sim], remappings=cmd_vel),
        Node(package='nav2_planner', executable='planner_server', name='planner_server', output='screen',
             parameters=[params, sim]),
        Node(package='nav2_route', executable='route_server', name='route_server', output='screen',
             parameters=[params, sim, {'graph_filepath': arg('graph')}]),
        Node(package='nav2_behaviors', executable='behavior_server', name='behavior_server',
             output='screen', parameters=[params, sim], remappings=cmd_vel),
        Node(package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator', output='screen',
             parameters=[params, sim, {'default_nav_to_pose_bt_xml': share('behavior_trees', 'navigate_on_route.xml')}]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_navigation',
             output='screen', parameters=[sim, {'autostart': True, 'node_names': NAV2_NODES, 'bond_timeout': float(arg('bond_timeout'))}]),
    ]
    graph_viz = Node(
        package='aplc_navigation', executable='route_graph_viz', name='route_graph_viz', output='screen',
        parameters=[sim, {'graph_filepath': arg('graph')}],
    )
    rviz = Node(
        package='rviz2', executable='rviz2', output='screen', condition=IfCondition(arg('rviz')),
        arguments=['-d', share('rviz', 'route_demo.rviz')], parameters=[sim],
    )
    # Nav2 waits for TF (map -> odom needs the EKF and the Quest), so give Gazebo a head start.
    return [world, quest, TimerAction(period=float(arg('nav2_delay')), actions=nav2), graph_viz, rviz]


def generate_launch_description():
    d = lambda name, default, desc: DeclareLaunchArgument(name, default_value=default, description=desc)
    return LaunchDescription([
        d('world', 'house_1.sdf', 'Installed APLC world.'),
        # Default spawn: on the landing outside the front door, facing down the ramp.
        d('x', '0.7', 'Spawn x (m, map = Gazebo world).'),
        d('y', '3.5', 'Spawn y (m).'),
        d('z', '0.50', 'Spawn height of base_footprint (m); landing floor is 0.45.'),
        d('yaw', '-1.5708', 'Spawn yaw (rad).'),
        d('headset_x', '0.15', 'Headset (and depth camera) mount, forward of base_footprint (m).'),
        d('headset_height', '1.0', 'Headset (and depth camera) mount height above the ground (m).'),
        d('headset_pitch', '0.6', 'Depth camera downward tilt (rad); 0.6 sees the ground from ~0.5 m.'),
        d('camera_width', '424', 'Depth camera width (px).'),
        d('camera_height', '240', 'Depth camera height (px).'),
        d('camera_rate', '15', 'Depth camera rate (Hz).'),
        # world.launch.py's default (1 deg/s, an uncalibrated BMF055) turns the odom frame
        # by ~1 rad a minute; the local costmap lives in odom, so obstacles smear around
        # the robot. 0.1 deg/s is a gyro calibrated at startup.
        d('imu_gyro_bias', '0.0017453', 'Gyro bias magnitude per axis (rad/s).'),
        d('quest_noise', '0.004', 'Quest position noise stddev (m).'),
        d('quest_drift_max', '0.10', 'Largest Quest drift between anchor re-alignments (m).'),
        d('quest_dropout_period', '0', 'Seconds between simulated 1 s tracking losses; 0 = never.'),
        d('graph', PathJoinSubstitution([FindPackageShare('aplc_navigation'), 'graphs', 'house_route.geojson']),
          'Route graph (GeoJSON).'),
        d('gui', 'true', 'Open the Gazebo window.'),
        d('rviz', 'true', 'Open RViz with the route demo view.'),
        d('render_engine', 'ogre', 'Gazebo sensor render engine (ogre for the VM).'),
        d('seed', '', 'Random seed for Gazebo and the Quest error model; empty = random.'),
        # The lifecycle manager shuts Nav2 down when a node misses heartbeats this long. On
        # the 4-core VM the controller can stall that long under load; 0 disables the check.
        d('bond_timeout', '0.0', 'Nav2 bond timeout (s); 0 = off.'),
        d('nav2_delay', '8.0', 'Seconds to wait before starting Nav2.'),
        OpaqueFunction(function=setup),
    ])
