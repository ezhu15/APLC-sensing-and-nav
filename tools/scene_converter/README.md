# APLC scene converter

Deterministic Python 3.10+ conversion, with no runtime dependencies on ROS,
Unreal, MRUK, AI, or a connected headset. Run from this directory, or install
with `python3 -m pip install .` to obtain the `aplc-scene` command.

```bash
python3 -m aplc_scene import-quest /path/to/quest-snapshot.json --output building.scene.json
python3 -m aplc_scene validate building.scene.json
python3 -m aplc_scene gazebo building.scene.json --output /path/to/new-gazebo-export
python3 -m unittest discover -s tests -v
```

Input to `import-quest` is the custom APLC schema-1 room JSON, **not** the
unmodified `.mruk.json`. Gazebo export accepts only the portable map; it never
reinterprets its coordinates as Unreal units. Existing files/directories are
refused to protect previous exports and annotations.

## Portable map v1

`format: aplc.scene`, `schema_version: 1`. Geometry uses meters in one
right-handed capture frame: X forward, Y left, Z up. `frame_id` defaults to `map`
and is configurable with `--frame-id`. That name does not establish alignment
with an existing ROS map. `alignment.status: unresolved` explicitly records this.

- `map_id`: SHA-256 identity of the imported source snapshot, not a survey ID or
  a checksum of later operator annotations. Version annotated files separately.
- `provenance`: source hash, adapter name, original capture/tracking metadata,
  and basis conversion. Metadata is explicitly source-native; geometry is SI.
- `rooms`: UUID, optional human name, optional floor ID, room pose, anchor IDs.
  A room is not automatically a building floor. Null names/floor IDs await annotation.
- `anchors`: UUID, all room memberships, labels, position/orientation/scale,
  transformed plane boundary and unit normal, chosen rendering geometry.
- `geometry`: null, a plane boundary in world meters, or eight transformed box
  corners in the adapter's stable X/Y/Z min/max enumeration order. A volume's
  original plane is preserved separately when present. Scale has already been
  applied to geometry and must not be applied again by consumers.
- `annotations`: reserved `floors` and `connections` lists. No automatic stair
  pairing, floor grouping, or stair shape inference is performed in this version.

The adapter reflects Unreal Y and divides by `world_to_meters`; quaternion
conversion is `(-qx, qy, -qz, qw)`. Anchor world transforms are applied exactly
once; room transforms are not multiplied into them. Empty-geometry anchors are
retained, and identical shared anchors retain all room memberships. Conflicting
UUIDs are rejected. Separate sessions are not merged or aligned.

## Gazebo output

`world.sdf`, `<name>/model.sdf`, `model.config`, OBJ meshes with flat normals,
`anchors.json`, and `source_map.json`. Model defaults to `aplc_rooms`.

Planes receive an assumed centered 0.02 m thickness, configurable with
`--plane-thickness`. Volume boxes retain their measured bounds. WALL_ART,
WINDOW_FRAME, DOOR_FRAME and OPENING have visual geometry but no collision;
other solid labels use the existing converter collision policy. No artificial
stairs are generated from landmark labels. Parent walls/floors remain solid:
door openings and stair holes are not cut. Review before robot navigation.

Use the existing repository Docker image for headless validation:

```bash
docker run --rm --user "$(id -u):$(id -g)" \
  -v /absolute/export:/scene:ro \
  -e GZ_SIM_RESOURCE_PATH=/scene -e SDF_PATH=/scene aplc-dev:jazzy \
  bash -c 'source /opt/ros/jazzy/setup.bash; gz sdf -k /scene/world.sdf && gz sim -s -r --iterations 20 /scene/world.sdf'
```

This converter does not publish ROS messages, change the pose protocol, perform
localization, or generate navigation occupancy grids. A future RViz publisher
can consume this portable map directly without importing the Unreal adapter.
