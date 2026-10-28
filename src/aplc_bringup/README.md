# aplc_bringup

Top-level launch for the APLC: RTAB-Map mapping and localization on top of the simulation
and the EKF. Full documentation: [docs/localization/rtabmap.md](../../docs/localization/rtabmap.md).

## Each person builds their own map

The RTAB-Map database and its alignment file are **not in git**. They are created on your
machine by a mapping drive:

| File | What it is |
|---|---|
| `maps/rtabmap.db` | The map: images, features and the pose graph from your drive |
| `maps/map_alignment.yaml` | Where the map frame sits in Gazebo's world, recorded at the start of that same drive; used only to compare the estimate with ground truth |

The two belong together: both are rewritten by every mapping run, in the same folder. Run
a mapping drive before localization or the demo; they stop with a "run mapping first"
message if either file is missing.

## Mapping

```bash
colcon build --symlink-install
source install/setup.bash
ros2 launch aplc_bringup mapping.launch.py
```

This starts Gazebo (with the robot at the driveway start point), the controllers, the EKF,
`rgbd_sync`, RTAB-Map in mapping mode, the map alignment recorder and RViz showing the
growing occupancy grid. The previous database in `maps/` is deleted at start.

Drive with Gazebo's **Drive APLC** panel (Keyboard tab: W/S forward/back, A/D turn), or
from a second terminal:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \
  -p stamped:=true -p use_sim_time:=true -r cmd_vel:=/diff_drive_controller/cmd_vel
```

Stop with Ctrl-C in the launch terminal; RTAB-Map saves the database as it shuts down.

Arguments: `gui` (true), `rviz` (true), `rtabmap_viz` (false), `database_path`
(default `maps/rtabmap.db`; the alignment file goes in the same folder), `imu_gyro_bias`
(default 0.1 deg/s in rad/s), `seed`.
