# ROS 2 Unitree L2 LiDAR–internal-IMU calibration

This branch ports the LI-Init estimator to ROS 2 Humble. It runs separately from the FAST-LIO and Point-LIO evaluation nodes. The launch subscribes directly to Unitree DDS `rt/utlidar/cloud` and `rt/utlidar/imu` through the paired bridge and republishes them as `/li_init/input/cloud` and `/li_init/input/imu`. Both ROS message headers keep the original L2 sensor stamps, and cloud point `time` remains seconds relative to the cloud start stamp. The parser converts those seconds to internal milliseconds. LI-Init estimates the residual LiDAR/IMU time offset itself.

The ROS 2 package needs Ceres, PCL, Eigen, and the workspace's `livox_ros_driver2` message package. With system packages installed:

```bash
sudo apt install libceres-dev
cd /home/baixiao/Desktop/IsaacLab/source/unitree_mujoco/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select go2_dds_ros2_bridge lidar_imu_init
source install/setup.bash
ros2 launch lidar_imu_init go2_l2_calibration.launch.py
```

On this machine Ceres and its libraries are also unpacked under `~/.local/li_init_deps`, so a build without system installation works with:

```bash
CMAKE_PREFIX_PATH="$HOME/.local/li_init_deps/usr:$CMAKE_PREFIX_PATH" \
LD_LIBRARY_PATH="$HOME/.local/li_init_deps/usr/lib/x86_64-linux-gnu:$HOME/.local/li_init_deps/usr/lib:$LD_LIBRARY_PATH" \
colcon build --packages-select go2_dds_ros2_bridge lidar_imu_init \
  --cmake-args -DCeres_DIR="$HOME/.local/li_init_deps/usr/lib/cmake/Ceres"
```

The launch adds that local library path to the calibration process when it exists. To override hardware DDS settings, use `dds_interface:=...` or `dds_domain_id:=...`. To change the result location, use `output_dir:=/absolute/path`. The default is `~/.ros/li_init_l2`; the calibration report is `result/Initialization_result.txt` and diagnostics are under `Log/` there. Each run opens these files anew.

Leave the robot stationary for at least five seconds after launch, then provide varied safe rotations and translations until the printed X/Y/Z excitation indicators progress. Continue through the online refinement phase. A stationary run cannot calibrate all axes. The report labels its 4×4 matrix **LiDAR to IMU**: it maps cloud points into the L2 internal IMU coordinates. It also reports the IMU-to-LiDAR time lag. Compare repeated runs before copying those values into `fastlio_l2.yaml`; disable online FAST-LIO extrinsic estimation after the calibration is validated.

The LI-Init evaluation outputs are `/li_init/odom`, `/li_init/path`, `/li_init/cloud_registered`, and `/li_init/cloud_registered_body`, with TF `li_init_world → li_init_body`. The package uses the L2 `PointCloud2` fields `x`, `y`, `z`, `intensity`, `ring`, and `time`; the cloud header must mark scan start. `initialization.mean_acc_norm` in `config/go2_l2_ros2.yaml` is currently 11.09 m/s², measured from 1,253 live stationary L2 IMU samples with the head tilted up on 2026-10-15. Remeasure it when the head pose or sensor setup changes; earlier stationary data in a different pose implied about 11.93 m/s².
