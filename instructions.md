# Running the APLC simulation

How to launch the house world with the APLC robot, drive it by keyboard, and view the
RealSense camera. ROS 2 Jazzy on Ubuntu 24.04 with Gazebo Harmonic.

## 1. Install dependencies and build (once, and after pulling new packages)

From the repository root:

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src -y --ignore-src
colcon build --symlink-install
source install/setup.bash
```

`rosdep` installs everything declared in the `package.xml` files. For the camera that is:

| Package | Why it's needed |
|---|---|
| `ros-jazzy-realsense2-description` | RealSense camera model (mesh) and its frames: `camera_link`, `camera_depth_optical_frame`, `camera_color_optical_frame`, ... |
| `ros-jazzy-depth-image-proc` | Builds the camera point cloud from the color and depth images |
| `ros-jazzy-rclcpp-components` | Container that runs the point cloud node |
| `ros-jazzy-ros-gz-bridge` | Bridges the camera images and `/clock` from Gazebo to ROS |

The simulated camera itself needs no extra plugin: it is Gazebo's built-in `rgbd_camera`
sensor, run by the `Sensors` system already loaded in the worlds.

Drive control uses `ros-jazzy-gz-ros2-control`, `ros-jazzy-diff-drive-controller` and
`ros-jazzy-joint-state-broadcaster`; teleop uses `ros-jazzy-teleop-twist-keyboard`.
All are installed by the same `rosdep` command.

**Docker users:** add `COPY src/aplc_control/package.xml /tmp/dependencies/aplc_control/package.xml`
to `docker/Dockerfile` next to the other `COPY` lines and rebuild the image, otherwise the
controller packages are not installed.

## 2. Launch the simulation

```bash
ros2 launch aplc_gazebo world.launch.py
```

This one command:

1. Starts Gazebo (server and GUI) with the house world, **already running** (not paused).
2. Bridges `/clock` and the camera topics to ROS (`aplc_gazebo/config/bridge.yaml`).
3. Starts `robot_state_publisher` with the robot description (simulation time).
4. Spawns the parked RAV4 and the APLC robot.
5. Starts the wheel controllers (`joint_state_broadcaster`, `diff_drive_controller`) once
   the robot exists.
6. Starts the point cloud node (`depth_image_proc`).

Wait for this line before driving (about 5–25 s after the robot appears):

```
Configured and activated diff_drive_controller
```

### Launch arguments

Pass any of these as `name:=value`. Distances in meters, angles in radians.

| Argument | Default | What it does |
|---|---|---|
| `world` | `house_1.sdf` | `house_1.sdf` (furnished two-storey house) or `house_ramp.sdf` (solid house, exterior only) |
| `x`, `y`, `z` | `5.0`, `-3.0`, `0.25` | Robot spawn position. Default is on the driveway. Indoors the floor is at z = 0.45, so use `z:=0.70` |
| `yaw` | `3.14159` | Robot heading. Default faces the house |
| `car` | `true` | Spawn the parked RAV4 |
| `car_x`, `car_y` | `7.0`, `-3.4` | Car **rear bumper center**. The driveway spans x 2 to 14, y −5.15 to −1.65 |
| `car_yaw` | `0.0` | Car heading. `0` = trunk toward the house |
| `camera` | `true` | Add the RealSense camera |
| `camera_model` | `d435` | `d435`, `d435i` or `d455` |
| `camera_xyz` | `"0.2062 0 0.02"` | Camera mount (bottom tripod screw) in `base_link`. Default: centered on the front face |
| `camera_rpy` | `"0 0 0"` | Camera roll, pitch, yaw. Positive pitch tilts it down |
| `imu_xyz` | `"0 0 0"` | IMU (`imu_link`) position in `base_link`. Default: chassis center |
| `imu_rpy` | `"0 0 0"` | IMU roll, pitch, yaw in `base_link` |
| `imu_gyro_bias` | `0.017453` | Gyro bias per axis (rad/s), BNO085 raw typical 1 °/s. `0` = calibrated |
| `imu_accel_bias` | `0.784532` | Accel bias per axis (m/s²), BNO085 raw typical 80 mg. `0` = calibrated |
| `render_engine` | `ogre` | `ogre` works in VMs. `ogre2` is needed for camera noise (ogre ignores it) |
| `rviz` | `false` | `true` also opens RViz with the saved configuration (see section 5) |
| `paused` | `false` | `true` starts Gazebo paused; the controllers then activate only after you press Play |

Examples:

```bash
# Robot inside the house, facing the front door
ros2 launch aplc_gazebo world.launch.py x:=-1.2 y:=3.5 z:=0.70 yaw:=0

# Car parked further down the driveway, slightly angled
ros2 launch aplc_gazebo world.launch.py car_x:=9.0 car_y:=-2.8 car_yaw:=0.15

# D455 mounted higher and tilted down, with camera noise
ros2 launch aplc_gazebo world.launch.py camera_model:=d455 \
  camera_xyz:="0.15 0 0.10" camera_rpy:="0 0.2 0" render_engine:=ogre2

# No car, no camera (lighter)
ros2 launch aplc_gazebo world.launch.py car:=false camera:=false

# IMU with no bias (ideal on-chip calibration), noise only
ros2 launch aplc_gazebo world.launch.py imu_gyro_bias:=0 imu_accel_bias:=0
```

Arguments that need spaces (`camera_xyz`, `camera_rpy`, `imu_xyz`, `imu_rpy`) must be quoted.

## 3. Drive the robot (teleop)

In a **second terminal**:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \
  -p stamped:=true -p use_sim_time:=true -r cmd_vel:=/diff_drive_controller/cmd_vel
```

Keep that terminal focused while driving.

| Key | Motion |
|---|---|
| `i` | forward |
| `,` | backward |
| `j` / `l` | turn left / right in place |
| `u` / `o` | forward while turning left / right |
| `k` | stop |

Hold keys down: the robot brakes 0.5 s after the last command. Speed is limited to
0.5 m/s and 1.0 rad/s. From the default spawn, drive toward the house, follow the apron to
the foot of the ramp (around x 0.6, y −3.6), go up the ramp, then turn left through the door.

Why the flags: `stamped:=true` because the controller expects `TwistStamped`;
`use_sim_time:=true` so the command timestamps match the simulation clock (otherwise the
0.5 s brake timeout never triggers).

## 4. View the camera

Images:

```bash
ros2 run rqt_image_view rqt_image_view /camera/camera/color/image_raw
```

Switch the dropdown to `/camera/camera/aligned_depth_to_color/image_raw` for depth.

Point cloud and robot in RViz:

```bash
rviz2
```

Set **Fixed Frame** to `odom`, then **Add** → `RobotModel` (Description Topic
`/robot_description`), `TF`, and `PointCloud2` (Topic `/camera/camera/depth/color/points`).

Camera topics (named like the real `realsense2_camera` driver with depth alignment on, so
code written in simulation works on the hardware):

| Topic | Type | Notes |
|---|---|---|
| `/camera/camera/color/image_raw` | `sensor_msgs/Image` (rgb8) | 848×480, 30 Hz |
| `/camera/camera/color/camera_info` | `sensor_msgs/CameraInfo` | |
| `/camera/camera/aligned_depth_to_color/image_raw` | `sensor_msgs/Image` (32FC1, meters) | Real driver: 16UC1 millimeters |
| `/camera/camera/aligned_depth_to_color/camera_info` | `sensor_msgs/CameraInfo` | |
| `/camera/camera/depth/color/points` | `sensor_msgs/PointCloud2` (xyz + rgb) | Built by `depth_image_proc` |

All are in the `camera_depth_optical_frame` (z forward, x right, y down).

Quick checks:

```bash
ros2 topic hz /camera/camera/color/image_raw        # ~30 Hz
ros2 control list_controllers                        # both controllers "active"
```

## 5. Open RViz with the saved configuration

The saved configuration (`src/aplc_gazebo/rviz/aplc_sim.rviz`) opens with the robot model,
TF, the depth point cloud (`/camera/camera/depth/color/points`, in its true colors) and the
color image panel already set up. Fixed Frame is `odom`, and the view follows the robot.

With the simulation running, in another terminal:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch aplc_gazebo rviz.launch.py
```

Or start the simulation and RViz together:

```bash
ros2 launch aplc_gazebo world.launch.py rviz:=true
```

To use your own configuration: `ros2 launch aplc_gazebo rviz.launch.py rviz_config:=/path/to/file.rviz`.
If you change the view and want to keep it, use **File → Save Config As** and pass that file.

## Other robot topics

| Topic | Type |
|---|---|
| `/diff_drive_controller/cmd_vel` | `geometry_msgs/TwistStamped` (input) |
| `/diff_drive_controller/odom` | `nav_msgs/Odometry` (wheel odometry) |
| `/joint_states` | `sensor_msgs/JointState` (wheel encoders) |
| `/imu` | `sensor_msgs/Imu`, frame `imu_link`, 100 Hz. Details: [docs/sensors/imu.md](docs/sensors/imu.md) |
| `/tf`, `/tf_static` | `odom → base_link → wheels, camera frames` |
| `/clock` | simulation time |

## Troubleshooting

| Symptom | Cause / fix |
|---|---|
| Robot doesn't move; `ros2 control list_controllers` times out | Simulation is paused (`paused:=true`, or Pause was pressed). Press ▶ Play in Gazebo |
| Teleop runs but nothing happens | All terminals must use the same `ROS_DOMAIN_ID` and `ROS_AUTOMATIC_DISCOVERY_RANGE`. After changing them, close everything and run `ros2 daemon stop` |
| Gazebo GUI crashes with `libpthread.so.0: undefined symbol` | Launched from VS Code's integrated terminal (snap). Use a regular terminal |
| `colcon build` fails with `No module named 'catkin_pkg'` | A conda environment is active. Run `conda deactivate` |
| No point cloud | `depth_image_proc` not installed: rerun the `rosdep install` command |
| Camera noise has no effect | Use `render_engine:=ogre2` |
