# Guided Go2 body IMU calibration

Source ROS 2 Humble and the workspace install setup, then start:

```bash
ros2 launch lidar_imu_init go2_body_imu_static_calibration.launch.py mode:=full
```

This starts a dedicated Go2 `rt/lowstate` IMU bridge, publishing
`/go2_body_imu_calibration/imu` with frame `body`. It does not require FAST-LIO
and does not change `/imu/data_fastlio` or any SLAM configuration. The browser
UI opens at `http://127.0.0.1:8766/` when a desktop session is available. The
console prints the actual URL. Results go to a new directory under
`~/.ros/go2_body_imu_static_calibration/` for each run.

To use the existing body IMU stream instead of starting another DDS subscriber:

```bash
ros2 launch lidar_imu_init go2_body_imu_static_calibration.launch.py \
  start_bridge:=false imu_topic:=/imu/data_fastlio mode:=full
```

The IMU topic must contain `sensor_msgs/msg/Imu` in the `body` frame. Choose
`mode:=accelerometer` or `mode:=gyroscope` as needed. For gyro-only calibration,
pass `accelerometer_calibration:=/absolute/path/to/calibration.yaml`, or select
a passing body-IMU accelerometer result from the UI. The UI searches only this
launch's `output_dir`, so L2 internal IMU results do not appear in its list.

To see the live measured acceleration vector:

```bash
rviz2 -d "$(ros2 pkg prefix lidar_imu_init)/share/lidar_imu_init/config/go2_body_imu_vector.rviz"
```

The arrow is published on `/go2_body_imu_calibration/acceleration_vector` in
the `body` frame. It has one metre of length per 9.81 m/s². This view uses
`body` as RViz's fixed frame, so no FAST-LIO transform is needed.

The six stationary pose instructions, acceptance conditions, correction model,
and result format are the same as in `l2_imu_static_calibration.md`.

Gyro prompts refer to the **robot body axes**: X forward, Y left, Z up. Begin
with the body flat and hold still. Pitch about body Y, then hold. Return to
the initial flat orientation and hold; this return motion is deliberately
discarded. Then roll about body X from flat and hold. Finally, while tilted,
twist about the robot's own body Z axis and hold. The software checks the
measured gravity-direction change (at least 20°) and integrates the body IMU
gyro data between stationary endpoints. During the first two training
rotations it also checks whether the gravity change is predominantly along the
expected body axis. A level spin of the chair about room vertical does not
provide the required gravity change.
Aim to finish the roll at a manageable 30–45° overall tilt. A roughly 45–60°
twist about body Z from that tilt can provide the required change; pitching the
robot up 90° is unnecessary.
If the first pitch was already close to 90° head-up, lower the head to the
baseline flat pose and hold there before the roll. Pure roll at 90° head-up is
poorly observable because body X is nearly parallel to gravity. The flat
return must be within 12° of the corrected acceleration direction recorded
during the initial baseline and remain stationary for the configured hold.
