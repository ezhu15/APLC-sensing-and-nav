# Validation

Validated against the existing Quest 2 snapshot captured 2026-11-04.

- Nine Python unittest cases pass: concave triangulation, basis conversion,
  multilevel geometry/normals/collision policy, native JSON rejection, portable
  round trip/metadata, shared membership/conflicts, volumes/missing geometry,
  invalid input and single-application frame conversion.
- Real snapshot: 12 rooms, 142 retained anchors, 142 meshes, 121 colliders.
- Maximum converted boundary error against independently exported world-space
  boundary points: 5.62e-15 meters.

- Gazebo Harmonic in the existing `aplc-dev:jazzy` Docker image: model and world
  SDF validation passed; headless simulation completed 20 steps with exit 0.

No live headset query or pose protocol changes are required for this converter.
No RViz publisher, navigation map, stair inference or cross-session alignment
is included.
