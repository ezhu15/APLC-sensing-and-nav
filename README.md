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

### Prerequisites

Install ROS 2 Jazzy and the matching Gazebo/ROS integration packages. A Linux development environment is recommended.
With many of these details to be determined, I think it makes sense to support both a native ubuntu environment as well as docker images.

### Build

From the repository root (once the ROS 2 packages are populated):
> Note that this is a template section for now, and may change depending on build needs.

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### Run

Launch commands will be added once the robot description and simulation launch files are implemented.

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
