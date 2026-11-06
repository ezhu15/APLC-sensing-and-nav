# RViz scene publisher

Loads the portable `aplc.scene` map and publishes `visualization_msgs/MarkerArray`
on `/scene/markers` (reliable, transient-local, depth 1). Stable IDs and a leading
DELETEALL replace stale markers. The complete scene is republished every five
seconds and retained for late subscribers. Use one publisher per topic.

## Build and run in the existing Docker environment

From the repository root:

```bash
bash scripts/docker-dev.sh run --rm dev bash -c 'source /opt/ros/jazzy/setup.bash; colcon build --symlink-install --packages-select aplc_scene_ros'
bash scripts/docker-rviz.sh
```

The RViz wrapper reuses the existing display authentication and renderer selection.
Pass `--renderer software`, `mesa`, or `nvidia` as needed. To choose another map:

```bash
bash scripts/docker-rviz.sh --renderer software map:=/ws/maps/my-building.scene.json
```

Native ROS Jazzy:

```bash
source install/setup.bash
ros2 launch aplc_scene_ros view.launch.py
ros2 launch aplc_scene_ros view.launch.py rviz:=false show_ceilings:=true
```

Parameters: `map_file` (required for direct `ros2 run`), `show_ceilings` (false),
`show_labels` (true), `republish_seconds` (5.0). Parameters are read at startup;
restart to reload a map. Launch arguments expose `map`, `rviz`, `show_ceilings`,
and `show_labels`.

Floors are green, landmark surfaces orange, other surfaces blue-gray. Room names
fall back to short room UUIDs. WALL_ART/WINDOW_FRAME receive semantic labels;
no up/down stair meaning is inferred. Empty-geometry anchors remain in the map
but have no surface marker. Boxes and planar surfaces are displayed without
Gazebo's artificial thickness. Ceilings are optional; room elevations are retained.

For live headset movement, see [Live Quest](../../docs/LiveQuest.md) and run
`bash scripts/docker-rviz.sh --live`.

The bundled RViz config uses fixed frame `map`. If the map specifies another
`frame_id`, set RViz's Fixed Frame to that value. The static scene publisher itself publishes no TF or robot motion; the optional
Quest bridge supplies live headset pose and TF.

## Integration check

After building and sourcing the workspace:

```bash
APLC_TEST_MAP=/ws/maps/quest_20261104.scene.json python3 src/aplc_scene_ros/test/check_publisher.py
```

This starts the actual publisher, connects a late ROS subscriber, and checks
retained delivery, room/geometry counts, stable IDs, frames and mesh vertices.

See [validation results](VALIDATION.md). Pure geometry tests can also run without ROS
using `PYTHONPATH=tools/scene_converter:src/aplc_scene_ros python3 -m unittest discover -s src/aplc_scene_ros/test`.

The launcher opens overview and headset POV windows. Use `pov:=false` for a
single window; see [POV instructions](../../docs/LiveQuest.md#overview-and-headset-pov-windows).
