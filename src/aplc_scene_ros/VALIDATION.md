# Validation — November 4, 2026

- Built `aplc_scene_ros` with colcon in the existing `aplc-dev:jazzy` image.
- Actual ROS DDS late-subscriber integration passed using the included map:
  130 geometry markers (ceilings hidden), 12 room labels, 17 landmark labels.
  Checked retained delivery, IDs, frame names, triangle counts and coordinates.
- Two pure geometry tests passed: concave plane area/elevation and box/empty data.
- Launch arguments resolved successfully from the installed package.
- Docker GUI launcher started both the publisher and RViz with NVIDIA OpenGL 4.6.
  No RViz errors appeared during the timed startup check. Visual placement was
  not independently measured against the physical building.
- Shell syntax and git whitespace checks passed.

No Quest session, pose sender or Gazebo geometry was changed. This publisher
provides static visualization, not localization or navigation.

## Live bridge — November 4, 2026

- ROS Jazzy colcon build passed in the existing Docker image.
- Nine geometry/protocol unit tests passed, including frame composition,
  numeric validation, map/frame mismatch, stale capture/receipt, replay/session
  handling and peer mismatch.
- Actual TCP-to-ROS integration passed on isolated ports: 15 known transformed
  poses, headset/world TF, trail, stale clearing and map-ID rejection.
- RViz started with NVIDIA OpenGL 4.6; its marker subscription was confirmed
  through ROS introspection. Quest input reached map-framed pose, TF and trail.
- Initial 20-second observation: 230 poses before USB forwarding disappeared;
  bridge reported alignment_disconnected and cleared the path as expected.
  No cause for the USB transport reset is established by this check.
- After restoring adb reverse rules, the bridge recovered without restarting:
  292 poses over 15.03 seconds (~19.4 Hz), only live status, zero rejected frames,
  both quest_world and quest_headset TF, 712 accumulated trail points.
- Real movement was present (sample axis spans ~0.13, 0.57, 0.15 m). Physical map
  registration and absolute tracking accuracy were not independently measured.
- Wi-Fi TCP reception was validated in the preceding network change; this live
  ROS/RViz hardware test used USB. Both use the same bridge listeners.
- Shell syntax and git whitespace checks passed. No Quest protocol/app changes
  were needed for this ROS bridge.

Evidence: `docs/validation/quest_bridge` in the repository. Run instructions:
[Live Quest](../../docs/LiveQuest.md). No robot navigation/controller is included.

## Second RViz headset POV — November 4, 2026

- Added and built the QuestPOV RViz view-controller plugin against installed
  ROS Jazzy. The final build completed without compiler warnings.
- Inherits the stock FPS camera basis and applies the target's full orientation
  as well as position, unlike stock frame-position-only tracking.
- Loaded `quest_pov.rviz` in a second RViz process with NVIDIA OpenGL 4.6 and no
  plugin-load errors. Confirmed two scene marker subscribers (overview and
  aplc_quest_pov) and two TF listener subscribers.
- Launch argument `pov` defaults true and can disable the extra window. The
  existing overview and bridge were left running while adding the POV viewer.
- No Quest streaming, alignment, sampling, or history changes in this feature.
- Evidence: docs/validation/quest_pov. Camera optics are illustrative; this is
  saved geometry rendered from the headset pose, not passthrough video.

## Retained history and wider POV

The running bridge was found to still have its old 1,000-point parameter; it
was restarted with the new 15,000-point default. Historical trail clearing on
transient status changes was replaced with retained segmented history. The
integration test now verifies a small point budget, history across stale input,
separate recovery segments, late-subscriber retained delivery, current-segment
Path semantics, and continued invalid-pose gating (17 mapped synthetic poses).
The POV plugin rebuilt with a configurable 75-degree vertical FOV.
Ten existing unit tests passed alongside the build. Evidence is in
`docs/validation/history_fov`.

## Reboot reconnect correction

The old bridge retained silent TCP clients indefinitely and refused replacement
connections. The hardware report showed no new messages for approximately
31 minutes after a Quest battery shutdown. Added a five-second connection lease
based on parsed messages, expiry logging, and connected-stream diagnostics.

Ten unit tests and both TCP/ROS integration checks passed. The reconnect check
leaves old sockets open, reconnects after lease expiry with a new session,
sequence zero and a reset device clock, and verifies live poses without a
bridge restart. The history/segmentation/map-rejection test also still passes.
The running bridge process was replaced once while leaving both RViz windows
open; it accepted both streams from 192.168.1.157 and reached live over Wi-Fi.
No headset app, FOV, sampling, or network port changes were needed.
Evidence: docs/validation/quest_reconnect.
