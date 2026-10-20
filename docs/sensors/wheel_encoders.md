# Wheel encoders (simulated)

Issue: #18. The APLC measures wheel motion with encoders on its four wheels. In
simulation there are two layers: Gazebo's exact wheel joint states, and a node that turns
them into what a real encoder reports (whole ticks, jitter, speed from tick counts).

## Where the data comes from

```
Gazebo wheel joints
  └─ gz_ros2_control (state interfaces: position, velocity)
       ├─ joint_state_broadcaster ─► /joint_states            exact angles and speeds
       │                                 └─ wheel_encoders node ─► /wheel_encoders   ticks, jitter
       └─ diff_drive_controller ─► /diff_drive_controller/odom
```

- **`/joint_states`** is published by `joint_state_broadcaster` from the simulated wheel
  joints. It is ideal: no quantization, no noise.
- **`/diff_drive_controller/odom`** is integrated by `diff_drive_controller` from the same
  wheel **positions** (`position_feedback: true`, `open_loop: false`). Rebuilding the
  odometry from `/joint_states` (wheel radius 0.1016 m, separation 0.4764 m, front and
  rear wheel averaged per side) reproduces it exactly.
- **`/wheel_encoders`** is published by the `wheel_encoders` node
  (`src/aplc_gazebo/scripts/wheel_encoders.py`), started by `world.launch.py`.

The odometry is computed from the ideal positions, not from `/wheel_encoders`. At 1024
ticks per revolution the difference is far smaller than wheel slip (see below).

## Topics

| Topic | Type | Rate | Content |
|---|---|---|---|
| `/joint_states` | `sensor_msgs/msg/JointState` | 50 Hz | Exact position (rad) and velocity (rad/s) of `front_left_joint`, `front_right_joint`, `rear_left_joint`, `rear_right_joint`. Effort is `nan` |
| `/wheel_encoders` | `sensor_msgs/msg/JointState` | 50 Hz | Same joints. Position = whole ticks × 2π / ticks_per_rev. Velocity from tick differences. Same timestamps as `/joint_states` |
| `/diff_drive_controller/odom` | `nav_msgs/msg/Odometry` | 50 Hz | Wheel odometry in `odom`, child frame `base_footprint`, with covariances |

The rate is the controller manager's `update_rate` (50 Hz) in
`src/aplc_control/config/controllers.yaml`.

## Encoder model (`/wheel_encoders`)

For each wheel, on every `/joint_states` message:

```
ticks    = floor(angle × ticks_per_rev / 2π + noise),   noise ~ N(0, tick_noise_stddev)
position = ticks × 2π / ticks_per_rev
velocity = (ticks_now − ticks_0.1s_ago) × 2π / ticks_per_rev / 0.1 s
```

- **Quantization**: the position is always a whole number of ticks.
- **Edge jitter**: near a tick boundary the count flickers by one, as real encoder edges do.
- **Speed from counts**: like a motor controller, speed comes from how many ticks arrived in
  a window, so it is grainy: resolution = one tick per window.

| Parameter | Launch argument | Default | Notes |
|---|---|---|---|
| `ticks_per_rev` | `encoder_ticks_per_rev` | `1024` | Ticks per **wheel** revolution (after any gearbox). Placeholder until the motors/encoders are chosen |
| `tick_noise_stddev` | `encoder_tick_noise` | `0.25` | Edge jitter, stddev in ticks |
| `velocity_window` | — | `0.1` s | Node parameter |
| — | `wheel_encoders` | `true` | `false` to not start the node |

At 1024 ticks/rev one tick is 0.352° of wheel rotation, 0.62 mm of travel, and the speed
resolution is 0.061 rad/s (6 mm/s).

## Odometry covariance

Set in `controllers.yaml` and published on `/diff_drive_controller/odom`. Diagonal order is
x, y, z, roll, pitch, yaw. Before #18 all were zero, which tells an EKF that wheel odometry
is perfect.

| | x | y | z | roll | pitch | yaw |
|---|---|---|---|---|---|---|
| `twist_covariance_diagonal` | 0.001 | 0.001 | 1e6 | 1e6 | 1e6 | **0.25** |
| `pose_covariance_diagonal` | 0.01 | 0.01 | 1e6 | 1e6 | 1e6 | 1e3 |

Why:

- **vx 0.001** (sd 0.03 m/s, about 6 % of the 0.5 m/s top speed): straight runs match
  ground truth exactly in simulation, but real floors slip; this leaves room for that.
- **vy 0.001**: a skid-steer robot barely moves sideways unless it is turning.
- **z, roll, pitch 1e6**: wheels don't measure these; the huge variance tells the EKF to
  ignore them.
- **yaw rate 0.25** (sd 0.5 rad/s): skid-steer turning slips a lot (next section). The
  error is systematic (always too large), which a covariance cannot describe, so the value
  is chosen to make the EKF rely on the gyro (`/imu`, variance 3e-6) for turning.
- **Pose**: `diff_drive_controller` publishes these as constants; they do not grow with
  distance travelled. The EKF should fuse the odometry **twist** (velocities), not the pose.
  Pose yaw 1e3 because wheel heading is unreliable after any turn.

## Measured slip (simulation, house_1.sdf driveway, asphalt)

**Headline number: in-place turns, odometry / truth = 1.49** (steady turning at 0.5 rad/s,
measured over a 14 s window: odometry 400.5°, truth 269.0°). Straight driving: 1.00.

Odometry and ground truth are always read **at the same instant**. Speeds are commanded on
`/diff_drive_controller/cmd_vel`. Whole-motion tests drive until the odometry reports the
target, then wait until the robot has stopped (the controller keeps turning for about a
second after the last command: 0.5 s brake timeout, then a 1 rad/s² deceleration ramp).

| Test | Odometry | Ground truth | Ratio |
|---|---|---|---|
| Straight, 0.3 m/s, 2 m | 2.094 m | 2.094 m | truth / odom = **1.000** |
| Steady spin, 0.5 rad/s, 14 s window | 400.5° | 269.0° | odom / truth = **1.489** |
| Spin to 90° (odom), 0.5 rad/s, whole motion | 97.98° / 112.30° | 62.90° / 76.53° | 1.558 / 1.467 (two runs) |
| Spin to 90° (odom), 1.0 rad/s, whole motion | 120.32° / 148.97° | 81.58° / 98.86° | 1.475 / 1.507 (two runs) |
| Spin to 90° (odom), −1.0 rad/s, whole motion | −120.32° | −81.58° | 1.475 |

- Straight driving: no measurable slip.
- Turning: the wheel odometry reports about **1.5×** the real rotation (1.49 steady; whole
  motions 1.40–1.56 because spinning up and slowing down slip differently, and runs vary by
  about ±0.05). The robot really turns only about two thirds of what the wheels say. Left
  and right turns are symmetric.
- An earlier quick test reported 1.24× (82.5° vs 66.8°). That was a measurement error: it
  read the odometry when the commands stopped but the ground truth after the robot had
  coasted to a stop. Repeating that exact test with both read at the same moment gives
  1.53 (at stop) and 1.56 (after stopping).
- Cause: on a four-wheel skid-steer robot the wheels must scrape sideways to turn, so the
  wheel speeds overstate the rotation. The effective wheel separation is about 1.5× the
  geometric 0.4764 m.

`wheel_separation_multiplier` stays at 1.0 for now. The measurement suggests about 1.49 for
in-place turns on this surface; whether to apply it is decided with the EKF (#9).

## How to verify

```bash
ros2 launch aplc_gazebo world.launch.py
```

In a second sourced terminal:

```bash
ros2 topic hz /joint_states                     # ~50 Hz
ros2 topic hz /wheel_encoders                   # ~50 Hz
ros2 topic echo /joint_states --once            # 4 wheels: position + velocity
ros2 topic echo /wheel_encoders --once          # same joints, quantized positions
ros2 topic echo /diff_drive_controller/odom --once --field twist.covariance   # nonzero diagonal
ros2 param get /diff_drive_controller position_feedback                       # True
```

Slip by eye: drive with teleop (see `instructions.md`), hold `j` until the odometry
yaw has changed by 90°, and compare with the robot in Gazebo:

```bash
ros2 topic echo /diff_drive_controller/odom --field pose.pose.orientation
gz model -m aplc -p                             # ground-truth pose
```

Results from the verification run: `/joint_states` 50.0 Hz (every stamp 20.0 ms apart),
`/wheel_encoders` 50.0 Hz with positions always whole ticks, encoder position within
−1.5 to +0.6 ticks of the exact angle, encoder speed 2.941 rad/s vs exact 2.948 rad/s
at 0.3 m/s (sd 0.077 rad/s).

## Sim-to-real gaps

- **Ticks per revolution is a placeholder.** Set `encoder_ticks_per_rev` to the real
  encoder's counts per wheel revolution (encoder CPR × 4 for quadrature × gear ratio) once
  the motors are chosen.
- **Slip depends on the floor.** These numbers are for the simulated driveway (friction
  0.8 on the ground plane). Carpet, tile and wood each resist the sideways scraping
  differently, so the real turn ratio will differ per surface and must be measured on the
  robot on each floor type. Indoor floors are not given their own friction in the world yet.
- **The drive will change.** The design is moving to tracks, which slip even more when
  turning. Re-measure when the tracked base is modelled.
- **Straight-line slip is zero in simulation.** Real wheels slip when accelerating, on
  ramps and on dusty or wet surfaces.
- **No wheel-radius error.** Real wheels differ in size, wear and compress under load, which
  causes drift when driving straight (`left_wheel_radius_multiplier` /
  `right_wheel_radius_multiplier` can model it later).
- **Odometry ignores the quantization.** `diff_drive_controller` integrates the ideal
  positions; on hardware it would integrate encoder counts.
- **No encoder faults**: no missed counts, glitches, direction errors or timing jitter.
- **Ideal timing**: every message is exactly 20 ms apart.
