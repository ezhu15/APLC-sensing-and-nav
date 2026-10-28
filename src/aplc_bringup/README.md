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
| `maps/aplc_map.pgm`, `.yaml` | 2D occupancy grid exported from that map (for Nav2), if you export it |

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
(default 0), `seed`.

**Gyro bias during mapping is 0**, which assumes the gyro bias is removed by a calibration
at startup (still to be built for the real robot). The EKF does not fuse wheel yaw, so any
gyro bias turns straight into heading drift: with 0.1 deg/s, a 10-minute drive left the
house rotated about 30 degrees in the map.

To keep the 2D grid for Nav2, export it while the mapping launch is still running:

```bash
cd src/aplc_bringup/maps
ros2 run nav2_map_server map_saver_cli -f aplc_map --ros-args \
  -p map_subscribe_transient_local:=true -p use_sim_time:=true -p save_map_timeout:=20.0 &
ros2 service call /rtabmap/publish_map rtabmap_msgs/srv/PublishMap "{global_map: true, optimized: true, graph_only: false}"
```

RTAB-Map only republishes `/map` when it changes, so the service call makes it send one for
the saver. This writes `aplc_map.pgm` and `aplc_map.yaml` (gitignored, like the database).
