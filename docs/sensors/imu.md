# IMU (simulated BNO085)

Issue: #17. The robot's IMU is a CEVA BNO085. In simulation it is a Gazebo `imu` sensor
with noise and bias taken from the datasheets, bridged to ROS as `sensor_msgs/msg/Imu`.

## What is simulated

| Quantity | Simulated | Notes |
|---|---|---|
| Angular velocity (gyroscope) | Yes | Gaussian noise + constant bias, per axis |
| Linear acceleration (accelerometer) | Yes | Includes gravity, like a real accelerometer. Gaussian noise + constant bias, per axis |
| Orientation | Yes, but ideal | Gazebo's orientation is noiseless, relative to the robot's pose at spawn, with zero covariance. See [Sim-to-real gaps](#sim-to-real-gaps) |
| Magnetometer | No | |
| BNO085 fused reports (rotation vectors, gravity, linear acceleration, step/stability detection) | No | Only the raw gyro/accel data an IMU driver would publish |

## Interface

| | |
|---|---|
| Topic | `/imu` |
| Type | `sensor_msgs/msg/Imu` |
| Frame | `imu_link` |
| Rate | 100 Hz (simulation time) |
| Gazebo topic | `/imu` (bridged in `src/aplc_gazebo/config/bridge.yaml`) |

`imu_link` is fixed to `base_link`. Its default position is the chassis center, which is
the robot's center of mass (the `base_link` origin; the wheels are placed symmetrically).
Axes follow `base_link`: x forward, y left, z up.

The sensor is defined in `src/aplc_description/urdf/aplc_sensors.xacro`. The world must
load the `gz::sim::systems::Imu` system; both `house_1.sdf` and `house_ramp.sdf` do.

## Noise parameters and their source

The BNO08X datasheet (CEVA, document 1000-3927 v1.17) gives the configured ranges in
section 6.6 (±8 g, ±2000 °/s) and states that the raw sensors are specified in the Bosch
**BMF055** datasheet. Its own performance table (Figure 6-14) only lists dynamic accuracy
of the fused, calibrated outputs (accelerometer 0.3 m/s², gyroscope 3.1 °/s), not noise
density or offsets. The values below therefore come from the BMF055 datasheet
(BST-BMF055-DS000-04, revision 0.4, November 2020).

| Datasheet parameter (typical) | Value | Page |
|---|---|---|
| Accelerometer output noise density | 150 µg/√Hz | 8 |
| Accelerometer zero-g offset | ±80 mg (±150 mg max) | 8 |
| Gyroscope output noise density | 0.014 °/s/√Hz | 9 |
| Gyroscope zero-rate offset | ±1 °/s (±3 °/s max) | 9 |

The gyroscope's rms "Output Noise" row is truncated in the published datasheet table; only
the density (0.014 °/s/√Hz) is readable, so that is what we use.

Converted to per-sample values at 100 Hz (stddev = density × √bandwidth, bandwidth =
50 Hz, the Nyquist frequency):

| Simulated parameter (each axis) | Value |
|---|---|
| Gyro noise stddev | 0.001728 rad/s (0.099 °/s) |
| Gyro bias | 0.017453 rad/s (1 °/s) |
| Accel noise stddev | 0.010402 m/s² |
| Accel bias | 0.784532 m/s² (80 mg) |

How Gazebo applies them:

- **Noise**: independent Gaussian noise on every sample.
- **Bias**: a constant offset of the configured magnitude on each axis, with a random sign
  chosen at startup. It stays fixed for the whole run (no drift).
- **Covariance**: Gazebo fills `angular_velocity_covariance` and
  `linear_acceleration_covariance` with stddev² on the diagonal (gyro 2.99e-6,
  accel 1.08e-4). The bias is **not** included in the covariance.

### Choosing the bias

The default bias is the raw sensor's typical offset. On the real robot the BNO085 runs
dynamic calibration that removes most of the zero-rate and zero-g offsets from its
calibrated reports, so the defaults are pessimistic. They are launch arguments:

```bash
# Ideal calibration: noise only
ros2 launch aplc_gazebo world.launch.py imu_gyro_bias:=0 imu_accel_bias:=0
```

With the default accel bias, a stationary robot reads about 9.8 ± 0.78 m/s² on z (the sign
changes between runs), and about ±0.78 m/s² on x and y.

## Launch arguments

| Argument | Default | Meaning |
|---|---|---|
| `imu_xyz` | `"0 0 0"` | `imu_link` position in `base_link` (m) |
| `imu_rpy` | `"0 0 0"` | `imu_link` roll, pitch, yaw in `base_link` (rad) |
| `imu_gyro_bias` | `0.017453` | Gyro bias magnitude per axis (rad/s) |
| `imu_accel_bias` | `0.784532` | Accel bias magnitude per axis (m/s²) |

Example: IMU mounted 10 cm above the chassis center, calibrated gyro:

```bash
ros2 launch aplc_gazebo world.launch.py imu_xyz:="0 0 0.10" imu_gyro_bias:=0
```

## Launch and verify

Start the simulation (see `instructions.md` for setup):

```bash
ros2 launch aplc_gazebo world.launch.py
```

In a second terminal (sourced), with the robot sitting still:

```bash
ros2 topic hz /imu                         # ~100 Hz
ros2 topic echo /imu --once                # frame_id: imu_link; nonzero covariance diagonals
ros2 topic echo /imu --field linear_acceleration    # z ~ 9.8 (+/- accel bias), small noise
ros2 run tf2_ros tf2_echo base_link imu_link        # imu_link in the TF tree
```

Turn in place with teleop (hold `j`) and watch the gyro:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \
  -p stamped:=true -p use_sim_time:=true -r cmd_vel:=/diff_drive_controller/cmd_vel
ros2 topic echo /imu --field angular_velocity.z     # in another terminal
```

Results from the verification run (default bias):

| Check | Expected | Measured |
|---|---|---|
| Rate | 100 Hz | 100.2 Hz |
| Frame | `imu_link` | `imu_link` |
| Gyro noise stddev (x, y, z) | 0.001728 rad/s | 0.00169, 0.00174, 0.00170 |
| Gyro mean at rest (x, y, z) | ±0.017453 rad/s | 0.0176, −0.0175, 0.0175 |
| Accel noise stddev (x, y, z) | 0.010402 m/s² | 0.0104, 0.0101, 0.0105 |
| Accel mean at rest (x, y, z) | ±0.785, ±0.785, 9.8 ± 0.785 m/s² | −0.784, 0.784, 10.585 |
| Turning (`j`, 1.0 rad/s commanded): ground-truth yaw rate | — | 0.644 rad/s |
| Turning: IMU `angular_velocity.z` | truth + gyro bias | 0.661 rad/s |
| Turning: wheel odometry `twist.angular.z` | — | 0.992 rad/s (skid-steer slip) |

The turn shows why the IMU matters for localization: wheel odometry overestimates the turn
rate by about 50 %, while the gyro is off only by its bias.

Gazebo's gravity is 9.8 m/s² (its default), so a level, still robot with zero bias reads
9.8 on z, not 9.81.

## Sim-to-real gaps

- **Orientation is ideal.** Gazebo publishes perfect orientation relative to the spawn pose
  with all-zero `orientation_covariance`, and has no orientation noise option. The real
  BNO085 reports orientation from its own sensor fusion with errors of a few degrees
  (datasheet Figure 6-14: Game Rotation Vector 1.5–2.5° non-heading error,
  0.5°/min heading drift). An EKF must not fuse this orientation as-is; either leave
  orientation out of `imu0_config` or add realistic covariance first.
- **No bias drift.** The bias is constant for a run. Real MEMS bias changes with
  temperature (BMF055: gyro ±0.015 °/s per K, accel ±1 mg/K typical) and over time.
- **No magnetometer**, so no absolute heading.
- **Bias is not in the covariance.** Filters that trust the covariance will not account for
  the constant offset.
- **No scale-factor, misalignment or cross-axis errors** (BMF055 gyro sensitivity tolerance
  ±1 % typical).
- **No on-chip calibration or fused reports**: the BNO085's calibration status, rotation
  vectors and its gravity/linear-acceleration split are not simulated.
- **No vibration model.** The only vibration is what the physics engine produces from the
  wheels and contacts.
- **Ideal mounting**: rigid, exactly aligned with `base_link` unless `imu_rpy` is set.

## Sources

- CEVA, *BNO08X Datasheet*, document 1000-3927 v1.17:
  <https://www.ceva-ip.com/wp-content/uploads/BNO080_085-Datasheet.pdf>
- Bosch Sensortec, *BMF055 Data sheet*, BST-BMF055-DS000-04, revision 0.4, November 2020:
  <https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmf055-ds000.pdf>
