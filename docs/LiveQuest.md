# Live Quest position in RViz

The `aplc_scene_ros/quest_bridge` node receives the unchanged Quest TCP streams:
pose (5005) and anchor alignment (5006). It publishes the headset in the same
map frame used by the portable scene publisher. No Unreal build changes are
needed for this bridge; use the previously installed alignment/network APK.

## Run with Docker

From the APLC repository root:

```bash
bash scripts/docker-dev.sh run --rm dev bash -c 'source /opt/ros/jazzy/setup.bash; colcon build --symlink-install --packages-select aplc_scene_ros'

# USB: headset connected and authorized; restarts the app and restores forwards.
python3 scripts/quest-network.py usb --restart
bash scripts/docker-rviz.sh --live
```

`--live` starts the bridge and exposes both TCP ports automatically. Only one
receiver can own these ports. Stop diagnostic/other pose receivers before starting.
Use `--renderer software` if needed. Close the launch with Ctrl+C in its terminal.
Static-only RViz remains available without `--live`.

Wi-Fi uses the same bridge and launch command:

```bash
python3 scripts/quest-network.py wifi --host YOUR_COMPUTER_LAN_IP --restart
bash scripts/docker-rviz.sh --live
```

ADB is used for initial mode selection; USB can then be disconnected in Wi-Fi
mode. See [transport setup](QuestNetworking.md). If USB reconnects and forwarding
is lost, rerun the USB setup command or restore both `adb reverse` rules.

Native ROS Jazzy:

```bash
source install/setup.bash
ros2 launch aplc_scene_ros view.launch.py live:=true
# Or headless:
ros2 launch aplc_scene_ros view.launch.py live:=true rviz:=false
```

The launch's `map` argument selects the portable map. The headset must use the
same map ID/reference map. A custom `frame_id` also requires changing RViz's
Fixed Frame. `bind_address`, `pose_port`, and `alignment_port` launch arguments
configure listeners; defaults are 0.0.0.0, 5005 and 5006. For direct node use,
`map_file` is required; optional `pose_timeout` defaults to 0.5 seconds and
`path_limit` to 15000 poses (maximum 15000). ROS `use_sim_time` should remain false
for hardware testing; this bridge uses wall-time networking and a real Quest.

## Output

| Topic / TF | Meaning |
| --- | --- |
| `/quest/pose` (`PoseStamped`) | Headset position/orientation in the portable map frame |
| `/quest/path` (`Path`) | Current continuous trail segment, published up to 5 Hz |
| `/quest/trail` (`MarkerArray`) | Retained segmented history, up to 15,000 points total |
| `/quest/markers` (`MarkerArray`) | Cyan headset arrow; non-live status replaces it when unavailable |
| `/quest/status` (`String`, JSON) | Live/waiting/stale/rejection status, map/session ID, counters |
| `map -> quest_world` | Current anchor alignment (actual parent is the map frame ID) |
| `quest_world -> quest_headset` | Incoming headset pose |

No `map -> odom`, `base_link`, robot mount calibration, occupancy map, velocity
estimate, navigation controller, or covariance is invented. This is headset
visualization and TF, not autonomous navigation.

## Validation and stale data

The core verifies protocol version, numeric values, normalized quaternions,
sequence ordering, session, map/frame identity and >=3 reported inlier IDs.
The Unreal app performs anchor localization/consensus; ROS does not infer a new
alignment from raw geometry. The transform is applied exactly once:
`p_map = R_alignment * p_quest + t_alignment`.

A valid alignment must arrive with TTL <=2 seconds. Pose freshness defaults to
0.5 seconds. Both receipt age and capture age relative to the best observed
clock offset are checked. Device monotonic timestamps are never treated as ROS
wall timestamps. A relative clock-offset estimate is learned from both streams;
ROS pose/TF stamps are estimated capture times on the ROS clock. This detects
buffering relative to the best observed delay, but cannot establish absolute
one-way latency without clock synchronization.

Both connections must have the same source IP. Session changes clear previous
pose/trail state. Temporary interruptions preserve historical trail segments. Sequence reset/replay invalidates alignment; a new app session
is needed after a pose sequence reset. Invalid messages, disconnects, stale
streams, invalid alignment and map mismatches stop pose/TF publication and hide
the live arrow. Historical trail segments remain visible. The arrow also has a short lifetime. Previously sent TF
samples remain in ROS TF's historical cache; they are not fresh localization.
Consumers must check `/quest/status` and timestamps rather than extrapolate old TF.

Sockets are nonblocking, one active client per port. Input buffers/lines are
bounded (64 KiB / 16 KiB); malformed data is rejected. The bridge does not forward
scene geometry or modify the TCP protocol.

## Tests

After building and sourcing the workspace:

```bash
python3 -m unittest discover -s src/aplc_scene_ros/test -v
APLC_TEST_MAP=/ws/maps/quest_20261104.scene.json python3 src/aplc_scene_ros/test/check_live_bridge.py
```

The integration test uses isolated ports 15005/15006, real TCP, and actual ROS
subscriptions to check a known rotated/translated pose, TF, trail, stale handling
and map-ID rejection. It does not require the Quest.

See [validation results](../src/aplc_scene_ros/VALIDATION.md) and
[recorded ROS observations](validation/quest_bridge/recovery-observation.json).

## Camera controls

The saved RViz configuration includes Move Camera, Interact, Select, Focus Camera
and a Views panel. Select **Move Camera** in the toolbar, then left-drag to orbit,
middle-drag to pan, and use the wheel to zoom. The Views panel selects Orbit or
other camera types. If an older saved configuration has no camera tools, reload
`src/aplc_scene_ros/rviz/scene.rviz` or restart the launcher.

The Quest was subsequently switched to Wi-Fi at receiver 192.168.1.158 and the
live ROS bridge reached `live` without USB forwarding. USB remains selectable
with the transport setup helper. The camera-tool configuration omission was
corrected and the updated RViz instance launched without plugin-load errors.

### Stabilizing versus stale

`alignment_stabilizing` means the app most recently reported a candidate that
has not passed the temporal stability check. This status now expires after the
alignment TTL (at most two seconds), just like a valid alignment. With no fresh
updates the display changes to `alignment_stale`. `/quest/status` includes
`pose_age_seconds` and `alignment_age_seconds` to distinguish these cases.
A paused/stalled Quest app can leave TCP connections open without sending data;
wear/unlock the headset, foreground APLC, or restart it if updates do not resume.
Wi-Fi settings survive an app restart.

## Overview and headset POV windows

The normal launcher now opens two RViz windows:

```bash
bash scripts/docker-rviz.sh --live
```

The original Orbit view remains adjustable. The second window uses the
`aplc_scene_ros/QuestPOV` camera and follows `quest_headset` translation and full
orientation, including roll, with zero positional offset and a 5 cm near clip. Its default vertical field of
view is 75 degrees, adjustable under Views > Current View > Vertical FOV
(30–140 degrees).
It renders the saved scene, not passthrough video. Ceilings follow the existing
publisher setting and are hidden by default. The headset arrow, trail and grid
are hidden in the POV window to avoid obstructing its camera.

The custom camera is necessary because RViz Jazzy's stock FPS target-frame
tracking only updates position. The plugin inherits the FPS camera's ROS
X-forward/Z-up to Ogre camera basis, then applies the target's full orientation.
Mouse inputs do not offset the POV; use the overview window for free navigation.

To open only the original overview, pass `pov:=false`:

```bash
bash scripts/docker-rviz.sh --live pov:=false
```

The second RViz is a subscriber only: there is still one bridge and one scene
publisher. If tracking/alignment is unavailable, the POV may remain at the last
available TF pose. Check the overview's Quest status and `/quest/status`; a
stationary POV by itself does not prove live tracking.

Validation: plugin compiled and loaded in RViz using NVIDIA OpenGL 4.6 without
plugin errors. ROS introspection confirmed the POV window and overview both
subscribe to the scene. No Quest app, pose sampling, trail limit, or network
protocol changes were made for these views.

## Trail retention

The bridge now defaults to 15,000 points at the unchanged incoming pose rate
(about 12–13 minutes at 20 Hz). There is no time-based history expiration:
when full, only the oldest points are dropped. Brief tracking, alignment or
network interruptions preserve already recorded geometry. Recovery starts a new
line segment to avoid drawing a false straight line through an unobserved gap.
A new Quest app session or bridge restart clears in-memory history.

RViz's Quest trail display uses `/quest/trail`, a transient-local MarkerArray
with no marker lifetime, so reopening a viewer retains the existing bridge's
trail. `/quest/path` contains only the latest uninterrupted segment because a
single Path cannot represent breaks. The status JSON reports `history_points`
and `history_limit`. No sampling-rate reduction was introduced.

## Recovery after battery loss / headset reboot

The bridge closes a TCP connection after five seconds without a successfully
parsed protocol message (`connection_timeout`, configurable on the bridge).
This frees each receiver port even if the headset lost power without sending a
TCP close. A replacement connection can then be accepted. Normal brief pauses
still gate live pose publication immediately through the stricter freshness
checks; their recorded history remains visible.

A new app session resets the device-clock offset and sequence baseline and
requires a new valid map alignment. Test coverage includes leaving old sockets
open, reconnecting with sequence zero and a much smaller monotonic timestamp,
and recovering mapped ROS poses without restarting the bridge or RViz.
`/quest/status` also reports connected_streams and connection_idle_seconds.
The Quest app must be running and connected to Wi-Fi at the configured receiver
address. A new session still clears previous in-memory history by policy.
