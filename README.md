# Autonomous Personal Load Carrier (APLC)

A ROS 2–based project to develop and evaluate a four-wheel autonomous personal load carrier. The robot is intended to transport cargo, starting with manual driving and progressing toward assisted and autonomous navigation.

## Project goals

- Model a four-wheel mobile robot and its sensors in simulation.
- Support manual teleoperation and basic vehicle control.
- Simulate wheel odometry, IMU, depth camera, and proximity/safety sensors.
- Develop localization, obstacle detection, and navigation incrementally.
- Validate functionality in simulation before deploying on hardware.

## Technology stack

- **ROS 2 Jazzy** — robot software and message passing
- **Gazebo** — robot physics and sensor simulation
- **RViz 2** — visualization and debugging
- **Python / C++** — ROS nodes and algorithms
- **URDF/Xacro** — robot description

## Repository structure

The repository is expected to contain ROS 2 packages with responsibilities similar to:

```text
aplc/
├── README.md
├── docker/               # Dockerfile and Compose configurations
├── scripts/              # Host setup and container launch helpers
├── src/
│   ├── aplc_description/   # Robot model, meshes, and sensor frames
│   ├── aplc_bringup/       # Launch files and system configuration
│   ├── aplc_simulation/    # Gazebo worlds and simulation integration
│   ├── aplc_control/       # Drive control and teleoperation
│   ├── aplc_localization/  # Odometry and state estimation
│   ├── aplc_perception/    # Camera/depth processing and obstacle detection
│   └── aplc_navigation/    # Planning and autonomous navigation
└── ...
```

> This is a proposed layout; packages may be added or renamed as development progresses.

## Getting started

Choose Docker development below to build without installing ROS on the host,
or use a native Ubuntu environment with ROS 2 Jazzy and matching Gazebo/ROS
integration packages installed. Run host commands from the repository root.

### Docker development

On your Linux host, with Docker and Docker Compose installed, run as your
regular user (without sudo):

```bash
bash scripts/docker-host-setup.sh
bash scripts/docker-dev.sh build
bash scripts/docker-dev.sh run --rm dev
```

The setup script creates or updates the repository's `.env` with your host
user and group IDs so the container can write to the mounted workspace.
Other `.env` settings are preserved. Run it once per machine, or again if
your IDs change. `.env` is excluded from Git and the Docker build context.
Unset any previously exported `LOCAL_UID` or `LOCAL_GID` values if they are
incorrect: shell environment variables take precedence over `.env`.

The Dockerfile and Compose files live in `docker/`. `.dockerignore` stays at
the repository root (the build context) for compatibility with both classic
Docker builds and BuildKit. The repository is mounted at `/ws`. The scripts explicitly load
the root `.env` and select the Compose files, so no root Compose file is needed.
`docker-dev.sh` forwards its arguments to Compose (for example, `config` or
`build`). The project name stays `aplc-sensing-and-nav` for both launchers.

The image creates a named `dev` user with these IDs and `/home/dev` as its
home directory. Rebuild the image and start a new container after changing
the IDs or updating the Dockerfile; existing containers keep their old setup.

Inside the container:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### Gazebo GUI container

The separate `gazebo` service uses the same image and mounts the same repository
at `/ws`. Run the host setup and image build above first, then launch it from
your Linux desktop terminal:

```bash
bash scripts/docker-gazebo.sh
```

This starts Gazebo Harmonic's server and GUI with the selected house world
paused and spawns APLC at `(5, -3, 0.25)`. It uses X11
or XWayland (including a Wayland desktop), and requires `xauth` on the host.
The launcher copies the current display authentication cookie into a temporary
file, mounts it read-only, and removes it when the container exits. It does not
change your display server's access policy.

The launcher detects the local Linux host's GPU on each run and selects:

1. NVIDIA when `nvidia-smi` works and Docker has an `nvidia` runtime.
2. Intel/AMD Mesa using an available render node and its device group ID.
3. Software rendering when neither is detected (slower for complex scenes).

It prints the selected renderer. Override detection on hybrid machines or
when troubleshooting:

```bash
bash scripts/docker-gazebo.sh --renderer nvidia
bash scripts/docker-gazebo.sh --renderer mesa
bash scripts/docker-gazebo.sh --renderer software
```

`--gpu` remains an alias for `--renderer mesa`. NVIDIA uses
`docker/compose.gazebo.nvidia.yml`, including graphics/display driver capabilities
and PRIME offload settings. Mesa uses `docker/compose.gazebo.gpu.yml` and passes
through the first detected Intel/AMD render device.

On NVIDIA machines, install the host NVIDIA driver and follow the
[NVIDIA Container Toolkit installation and Docker configuration instructions](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)
first. This launcher selects container settings; it does not install host
drivers, change Docker daemon settings, or restart Docker. If NVIDIA is
detected but its Docker runtime is missing, auto mode explains the issue
and falls back to Mesa or software. Explicit NVIDIA mode fails with guidance.
Driver/runtime detection does not guarantee working OpenGL; if GUI startup
fails, inspect the error or try `--renderer software`.

This setup targets local Linux Docker Engine with X11/XWayland. Remote Docker
hosts, Docker Desktop/WSL, and rootless GPU configurations need separate setup.

To open a shell instead of starting the simulation:

```bash
bash scripts/docker-gazebo.sh bash
```

Inside that container:

```bash
source install/setup.bash
# Run the merged package's actual ros2 launch command here.
```

After merging `src/aplc-gazebo`, its files are immediately visible in both
containers. Add its package manifests to the Dockerfile's dependency COPY
step and rebuild the image to install any additional rosdep dependencies.
Then run `colcon build --symlink-install` in one container and source
`install/setup.bash` in each shell that will use the new packages. The ROS
package name comes from `package.xml`, not the folder name.

Both services share `build/`, `install/`, and `log/`; do not build concurrently
in the two containers. They also share the Compose network for ROS discovery;
use the same `ROS_DOMAIN_ID` if you set one. Sharing files alone does not bridge
Gazebo topics into ROS: the merged launch setup should start `ros_gz_bridge`
with the needed topic mappings. No APLC world or launch file is assumed until
that branch is merged.

### Stopping Docker containers

Exit an interactive container shell, or press Ctrl+C in the Gazebo launcher.
To stop and remove this project's containers, including one-off containers
created by the launch scripts, run on the host:

```bash
bash scripts/docker-dev.sh down --remove-orphans
```

The Docker image and files in the mounted workspace remain intact.

### Native build

With ROS 2 Jazzy, colcon, rosdep, and the required development tools installed,
resolve package dependencies and build from the repository root:

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

### Running APLC

After building and sourcing the workspace in a native ROS environment:

```bash
ros2 launch aplc_gazebo world.launch.py
# Or select the other installed world:
ros2 launch aplc_gazebo world.launch.py world:=house_ramp.sdf
```

### Docker and Gazebo GUI

From the repository root in a Linux desktop terminal, with Docker Compose
and `xauth` installed:

```bash
bash scripts/docker-host-setup.sh
bash scripts/docker-dev.sh build
bash scripts/docker-dev.sh run --rm dev bash -c 'colcon build --symlink-install'
bash scripts/docker-gazebo.sh
```

The host setup writes your UID/GID to the local `.env`. Docker configuration
lives in `docker/`; the scripts load `.env` and select the Compose files.
The separate Gazebo container shares the workspace at `/ws` and starts
`aplc_gazebo/world.launch.py` with `house_1.sdf`. Select the other world with:

```bash
APLC_WORLD=house_ramp.sdf bash scripts/docker-gazebo.sh
```

You can also set `APLC_WORLD` in `.env`. The GUI launcher automatically selects
NVIDIA, Intel/AMD Mesa, or software rendering. NVIDIA requires the host driver
and NVIDIA Container Toolkit configured for Docker. Override detection with
`--renderer nvidia`, `--renderer mesa`, or `--renderer software`.
GUI access uses X11/XWayland on the local Linux desktop.

For an interactive simulation shell:

```bash
bash scripts/docker-gazebo.sh bash
# Inside the container:
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch aplc_gazebo world.launch.py world:=house_ramp.sdf
```

Rebuild the image after dependency changes, and rebuild the workspace after
adding packages or launch files. Both containers share build outputs; do not
build concurrently. To stop this project's containers, including one-off runs:

```bash
bash scripts/docker-dev.sh down --remove-orphans
```

The world launch also starts the robot description publisher, bridges the
Gazebo clock to `/clock`, and spawns the base APLC model (without cargo) on
the driveway at world coordinates `(5, -3, 0.25)` meters with zero roll,
pitch, and yaw. Gazebo starts paused; wait for the robot to appear before
pressing Play. Gravity then lets it settle: `base_link` rests 0.203 m above
the ground, so its height will change from the initial spawn value.

Choose another spawn pose with the `x`, `y`, `z` (meters) and `yaw` (radians)
arguments. Indoors and on the landing the floor is at z = 0.45, so use
`z:=0.70`:

```bash
# Inside the house, facing the front door:
ros2 launch aplc_gazebo world.launch.py x:=-1.2 y:=3.5 z:=0.70 yaw:=0
# On the landing, facing down the ramp:
ros2 launch aplc_gazebo world.launch.py x:=0.6 y:=3.5 z:=0.70 yaw:=-1.5708
```

Do not launch a second description publisher alongside this world launch.
The preview joint-state publisher is disabled and simulation time is enabled.
Both worlds reference an SUV model on Gazebo Fuel; the first load needs
network access to download it.

### Driving the robot

After the spawn, the launch starts `joint_state_broadcaster` and
`diff_drive_controller` (configured in `aplc_control/config/controllers.yaml`).
The controllers can only activate while the simulation runs, so the spawner
waits up to 10 minutes for Play. Until then, `ros2 control` commands hang.

After pressing Play, drive by keyboard from a second terminal:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \
  -p stamped:=true -p use_sim_time:=true -r cmd_vel:=/diff_drive_controller/cmd_vel
```

Hold `i`/`,` to drive forward/backward and `j`/`l` to turn in place; `k`
stops. The robot brakes 0.5 s after the last command, so it stops when you
release the key. Keep `use_sim_time:=true`: with wall-clock timestamps that
timeout never triggers. Speeds are limited to 0.5 m/s and 1.0 rad/s.

While no command arrives, the launch logs "Velocity command timed out.
Braking." once a second; this is normal. The controllers publish
`/joint_states`, wheel odometry on `/diff_drive_controller/odom`, and the
`odom` → `base_link` transform. Wheel odometry overestimates turns because
the four wheels skid sideways.

## Development roadmap

| Phase | Target | Deliverables |
| --- | --- | --- |
| October 2026 | Simulation foundation | Robot description, Gazebo world, simulated drivetrain and sensors |
| November 2026 | Simulated Skills | Teleoperation, basic motion control, odometry, safety behavior |
| December 2026 | Beginning HWIL integration | Beginning stages of hardware integration, simulation using hardware sensors and collecting canned data |

## Planned sensors and safety systems

- Wheel encoders / wheel speed feedback
- Inertial measurement unit (IMU)
- RGB-D or stereo depth camera
- RGB Mono Camera
- Short-range obstacle and/or cliff detection
- Bump switches and emergency stop

Actual sensor selection and hardware interfaces are subject to change. Safety mechanisms must be validated on physical hardware before real-world use.

## Project status

**Early development — simulation-first.** Features described above are goals, not necessarily implemented functionality.

## Contributing

Use focused branches and pull requests for changes. Include a short description of the change and how it was tested. Document new ROS nodes, parameters, topics, and launch files as they are added.

Each pull request should be associated with a GitHub Issue to maintain traceability, encourage separation of concerns, and ensure changes remain focused on a specific feature, bug fix, or task.

Each issue GitHub issue should be directly attached to a branch labeled [issue #]-issue-name, for example: 12-create-basic-robot-description

## License

To be determined.
