# Guided L2 IMU calibration

Source ROS and the workspace `install/setup.bash`, then choose a mode:

```bash
# Accelerometer and gyroscope (default): 14 minimum steps
ros2 launch lidar_imu_init l2_imu_static_calibration.launch.py mode:=full

# Accelerometer only: 9 minimum steps
ros2 launch lidar_imu_init l2_imu_static_calibration.launch.py mode:=accelerometer

# Gyroscope only: 6 minimum steps, using an earlier accelerometer result
ros2 launch lidar_imu_init l2_imu_static_calibration.launch.py mode:=gyroscope \
  accelerometer_calibration:=/absolute/path/to/calibration.yaml

# Use an already-running IMU stream instead of starting a DDS bridge
ros2 launch lidar_imu_init l2_imu_static_calibration.launch.py \
  start_bridge:=false imu_topic:=/fastlio_l2/input/imu mode:=full
```

The default launch starts a separate L2 DDS bridge that publishes raw sensor-time
IMU data on `/l2_static_calibration/imu`. FAST-LIO and LI-Init do not need to run.
The launch opens a local browser page at `http://127.0.0.1:8765/` when a desktop
session is available. The console prints the actual URL. Set `open_browser:=false`
to open it manually, or set `ui_port:=<port>` if 8765 is in use. For a remote
robot computer, forward the printed port with `ssh -L 8765:127.0.0.1:8765
<robot-host>` and open the page on your own computer. The UI only listens on
loopback. Other launch arguments are `output_dir`, `dds_domain_id`, and
`dds_interface`.

## Guided session

Warm up the IMU. Securely support the robot and prevent balancing or motor motion.
The node only observes the IMU. Exact angles and a level head are unnecessary.
Skip any pose that cannot be safely supported. Status lines show overall progress
and the active phase, such as `[4/14 steps] Phase 2/5: Accelerometer fitting
(3/6 poses)`. A pose or rotation counts only after the node accepts it.

1. Lay the robot flat and hold still for 30 seconds for gyro bias and stationary
   noise. This baseline also counts as the first accelerometer pose.
2. Follow the remaining five pose prompts: robot head up, robot head down,
   roll left, roll right, and upside down. Hold each still for at least three
   seconds. Support the robot; the LiDAR head itself need not be level.
3. Hold two different poses for independent accelerometer validation.
4. Rotate slowly from one stationary tilt to another, then hold still for three
   seconds. Do this three times about varied axes, alternating directions.
   Each rotation needs at least 20 degrees of gravity-direction change. A spin
   around vertical alone does not constrain gyro scale.
5. Make two more rotations for independent gyro validation. Ctrl-C exits after
   completion. A failed validation stops with measurements preserved.

The node may request extra poses or rotations when the collected motions do not
constrain the fit. Keep each rotation, including endpoint holds, under 60 seconds.
Sensor timestamp gaps over 0.1 seconds, reverse time, or invalid
samples discard an interrupted rotation. Duplicate stamps are skipped. A frame change fails the session. Slow
constant movement can look stationary to an IMU, so physically hold each pose.

The page shows the current physical instruction, completed and remaining steps,
live IMU connection status, and saved result path. To continue after a previous
accelerometer run, choose its saved result under **Continue from an accelerometer
calibration** and press **Start gyro steps**. This saves the current run, creates
a fresh gyro-only result directory, and starts a new 30-second stationary
baseline. The dropdown lists passing accelerometer results in `output_dir`; the
live IMU frame must match the saved frame. You can also launch `mode:=gyroscope`
with `accelerometer_calibration:=...` directly.

To inspect the measured acceleration vector in RViz while calibrating, run
`rviz2 -d $(ros2 pkg prefix lidar_imu_init)/share/lidar_imu_init/config/l2_imu_vector.rviz`.
The orange arrow is published on `/l2_static_calibration/acceleration_vector`
in the IMU message frame, `l2_internal_imu` by default. It starts at the frame
origin; its direction is the measured specific-force vector and its length is
`|acceleration| / 9.81` metres. A stationary sensor with a normal gravity
reading therefore shows an approximately one-metre arrow. The arrow does not
show the robot's body orientation because the standalone calibration launch
does not publish a transform from this IMU frame to the robot body.

## Result and model

Each run creates a unique directory under `~/.ros/l2_imu_static_calibration/`
with `raw_imu.csv` and an atomically checkpointed `calibration.yaml`. Partial runs
retain the accepted data and stage. Restarting begins a new run.

The calibration estimates bias and separate scale for each accelerometer and gyro
axis. The correction matrices are diagonal; cross-axis misalignment and the
physical LiDAR-to-IMU rotation are not estimated:

```
a_corrected = C_acc @ (a_raw - bias_m_s2)
omega_corrected = C_gyro @ (omega_raw - bias_rad_s)
```

Gyro bias comes from the initial stationary hold and stays fixed while fitting
scale. Gyro integration uses sensor timestamps and matches the change in gravity
direction between stationary poses. Accelerometer data during rotation is not
used. Rotations about varied axes are needed to constrain all three gyro scales.

The fit requires a full-rank numerical Jacobian with condition number <=1000.
Matrix scale values must be between 0.5 and 2.0. Held-out validation requires
acceleration norm RMS/max error <0.15/0.4 m/s² and gyro gravity-direction RMS/max
error <2/5 degrees. Noise is reported as per-sample covariance at the measured
rate, with lag-one correlation; these are not continuous-time noise densities or
FAST-LIO noise parameters. Temperature dependence and long-term bias drift are
not measured. Results are not automatically applied to SLAM.

The ROS node also exposes `hold_seconds` (3), `baseline_seconds` (30),
`min_pose_angle_deg` (15, also a launch argument), `accel_std_limit` (0.35 m/s² per axis),
`accel_mean_shift_limit` (0.25 m/s² between halves of a hold),
`gyro_std_limit` (0.02 rad/s per axis), `gyro_norm_limit` (0.12 rad/s relative to
baseline bias), `max_gap_seconds` (0.1), `max_rotation_seconds` (60), `gravity`
(9.81), and minimum counts `target_poses` (6), `target_rotations` (3),
`validation_count` (2). Gyro-only mode requires a matching accelerometer result
file. Older files with a `fit` section are accepted if their quality check passes.
