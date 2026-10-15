#!/usr/bin/env python3
"""Guided accelerometer/gyro intrinsic calibration; never commands the robot."""
from datetime import datetime
from pathlib import Path
import csv
import os
from queue import Empty, SimpleQueue
from threading import Timer
import time
import webbrowser
import numpy as np
import yaml
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from lidar_imu_init.guided_imu_calibration import GuidedCalibration
from lidar_imu_init.imu_calibration_ui import start_ui_server
from lidar_imu_init.imu_static_calibration import validate_correction


DEFAULTS = dict(mode='full', imu_topic='/fastlio_l2/input/imu',
                accelerometer_calibration='', hold_seconds=3.0, baseline_seconds=30.0,
                accel_std_limit=0.35, accel_mean_shift_limit=0.25,
                gyro_std_limit=0.02, gyro_norm_limit=0.12,
                min_pose_angle_deg=15.0, target_poses=6, target_rotations=3,
                validation_count=2, gravity=9.81, max_gap_seconds=0.1,
                max_rotation_seconds=60.0,
                body_axis_guidance=False,
                ui_port=8765, open_browser=True,
                vector_topic='/l2_static_calibration/acceleration_vector',
                output_dir=str(Path.home() / '.ros/l2_imu_static_calibration'))


def load_accelerometer(path):
    report = yaml.safe_load(Path(path).expanduser().read_text())
    if not isinstance(report, dict):
        raise ValueError('Accelerometer calibration must be a YAML mapping')
    fit = report.get('accelerometer') or report.get('fit')
    if not isinstance(fit, dict):
        raise ValueError('File contains no accelerometer calibration')
    if fit.get('quality_pass') is False or fit.get('validation', {}).get('quality_pass') is False:
        raise ValueError('Accelerometer calibration failed quality checks')
    return fit, report


class StaticCalibration(Node):
    def __init__(self, parameter_overrides=None):
        super().__init__('imu_static_calibration', parameter_overrides=parameter_overrides)
        self.settings = {key: self.declare_parameter(key, value).value for key, value in DEFAULTS.items()}
        s = self.settings
        for key in ('hold_seconds', 'baseline_seconds', 'max_gap_seconds', 'gravity',
                    'accel_std_limit', 'accel_mean_shift_limit', 'gyro_std_limit',
                    'gyro_norm_limit', 'min_pose_angle_deg'):
            if not np.isfinite(s[key]) or s[key] <= 0:
                raise ValueError(key + ' must be finite and positive')
        if s['target_poses'] < 6 or s['target_rotations'] < 3 or s['validation_count'] < 2:
            raise ValueError('Need at least 6 training poses, 3 rotations, and 2 validation samples')
        if s['max_rotation_seconds'] <= 2 * s['hold_seconds']:
            raise ValueError('max_rotation_seconds must exceed twice hold_seconds')
        if not isinstance(s['ui_port'], int) or not 0 <= s['ui_port'] <= 65535:
            raise ValueError('ui_port must be between 0 and 65535')
        fit, source = None, None
        if s['mode'] == 'gyroscope':
            if not s['accelerometer_calibration']:
                raise ValueError('Gyroscope mode requires accelerometer_calibration:=<yaml>')
            fit, source = load_accelerometer(s['accelerometer_calibration'])
            if 'validation' not in fit:
                self.get_logger().warn('Legacy accelerometer file has no held-out validation; verify it independently.')
        self.source = source
        self.engine = GuidedCalibration(s, fit)
        self.frame = None
        self.last_receive = None
        self.last_log = 0.0
        self.saved_revision = -1
        self.last_acceleration = None
        self._accelerometer_cache = {}
        if source is not None:
            source_path = Path(s['accelerometer_calibration']).expanduser()
            source_stat = source_path.stat()
            source_choice = dict(session=source_path.parent.name,
                                 validation=('validated' if fit.get('validation', {}).get('quality_pass')
                                             else 'legacy; no held-out validation'),
                                 frame_id=source.get('frame_id'))
            self._accelerometer_cache[source_path] = (
                (source_stat.st_mtime_ns, source_stat.st_size), source_choice, fit, source)
        self._selection_queue = SimpleQueue()
        self._selection_message = ''
        self._open_session()
        self._ui_snapshot = self._make_ui_snapshot()
        source_html = Path(__file__).resolve().parents[1] / 'ui/imu_calibration.html'
        html_path = (source_html if source_html.exists() else
                     Path(get_package_share_directory('lidar_imu_init')) / 'ui/imu_calibration.html')
        self._ui_server, self.ui_url = start_ui_server(
            lambda: self._ui_snapshot, self._selection_queue.put, html_path.read_bytes(), s['ui_port'])
        self.get_logger().info(f'Calibration UI: {self.ui_url}')
        if s['open_browser'] and (os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY')):
            opener = Timer(0.7, lambda: webbrowser.open(self.ui_url, new=2))
            opener.daemon = True
            opener.start()
        self.create_subscription(Imu, s['imu_topic'], self.receive,
                                 QoSProfile(depth=4000, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.vector_publisher = self.create_publisher(Marker, s['vector_topic'], 10)
        self.last_vector_publish = 0.0
        self.create_timer(1.0, self.status)
        self.get_logger().info(f'Mode={s["mode"]}. '
                               f'{self.engine.status_line()} Warm up the IMU first. Securely support the robot; '
                               'prevent balancing/motor movement. Exact angles and a level head are unnecessary. '
                               f'Hold still for {s["baseline_seconds"]:.0f} seconds. Results: {self.output_path.parent}')
        self.save()

    def _open_session(self):
        folder = Path(self.settings['output_dir']).expanduser() / datetime.now().strftime('calibration_%Y%m%d_%H%M%S_%f')
        folder.mkdir(parents=True, exist_ok=False)
        self.output_path = folder / 'calibration.yaml'
        self.raw_file = (folder / 'raw_imu.csv').open('w', newline='')
        self.writer = csv.writer(self.raw_file)
        self.writer.writerow(['sensor_time_s', 'ax_m_s2', 'ay_m_s2', 'az_m_s2', 'gx_rad_s', 'gy_rad_s', 'gz_rad_s'])

    def _available_accelerometers(self):
        choices = []
        folder = Path(self.settings['output_dir']).expanduser()
        for path in sorted(folder.glob('calibration_*/calibration.yaml'), reverse=True)[:20]:
            if path == self.output_path:
                fit = self.engine.accelerometer
                if (self.engine.mode != 'gyroscope' and fit and
                        fit.get('validation', {}).get('quality_pass')):
                    choices.append(dict(session=path.parent.name,
                                        validation='validated', frame_id=self.frame))
                continue
            try:
                stat = path.stat()
                signature = (stat.st_mtime_ns, stat.st_size)
                cached = self._accelerometer_cache.get(path)
                if cached is None or cached[0] != signature:
                    fit, report = load_accelerometer(path)
                    validate_correction(fit['correction_matrix'])
                    choice = None
                    if fit.get('validation', {}).get('quality_pass', fit.get('quality_pass', False)):
                        bias = np.asarray(fit['bias_m_s2'])
                        if bias.shape == (3,) and np.all(np.isfinite(bias)):
                            choice = dict(session=path.parent.name,
                                          validation=('validated' if fit.get('validation', {}).get('quality_pass')
                                                      else 'legacy; no held-out validation'),
                                          frame_id=report.get('frame_id'))
                    self._accelerometer_cache[path] = (signature, choice, fit, report)
                    cached = self._accelerometer_cache[path]
                if cached[1] is not None:
                    choices.append(cached[1])
            except (OSError, ValueError, KeyError, TypeError):
                continue
        return choices

    def _make_ui_snapshot(self):
        acceleration = None
        acceleration_raw = None
        acceleration_norm = None
        if self.last_acceleration is not None:
            norm = float(np.linalg.norm(self.last_acceleration))
            if norm > 0:
                acceleration = (self.last_acceleration / norm).tolist()
                acceleration_raw = self.last_acceleration.tolist()
                acceleration_norm = norm
        return dict(progress=self.engine.progress(), instruction=self.engine.instruction(),
                    steps=self.engine.steps(), message=self.engine.message,
                    stage=self.engine.stage, mode=self.engine.mode,
                    connected=(self.last_receive is not None and time.monotonic() - self.last_receive < 2),
                    acceleration_direction=acceleration,
                    acceleration_raw_m_s2=acceleration_raw,
                    acceleration_norm_m_s2=acceleration_norm,
                    output_path=str(self.output_path),
                    available_accelerometers=self._available_accelerometers(),
                    selection_message=self._selection_message)

    def _handle_selection(self):
        try:
            session = self._selection_queue.get_nowait()
        except Empty:
            return
        choices = {c['session'] for c in self._available_accelerometers()}
        if session not in choices:
            self._selection_message = 'That accelerometer result is unavailable or failed validation.'
            return
        path = Path(self.settings['output_dir']).expanduser() / session / 'calibration.yaml'
        try:
            cached = self._accelerometer_cache.get(path)
            fit, source = (cached[2], cached[3]) if cached else load_accelerometer(path)
            if self.frame and source.get('frame_id') and source['frame_id'] != self.frame:
                raise ValueError(f'IMU frame mismatch: saved {source["frame_id"]}, live {self.frame}')
            self.save()
            self.raw_file.close()
            self.settings['mode'] = 'gyroscope'
            self.settings['accelerometer_calibration'] = str(path)
            self.source = source
            self.engine = GuidedCalibration(self.settings, fit)
            self.frame = None
            self.last_receive = None
            self.last_acceleration = None
            self._open_session()
            self.save()
            self._selection_message = f'Gyro session started from {session}. Hold still for the baseline.'
            self.get_logger().info(self._selection_message + f' New results: {self.output_path.parent}')
        except (OSError, ValueError, KeyError, TypeError) as exc:
            self._selection_message = f'Could not start gyro session: {exc}'
            self.get_logger().error(self._selection_message)

    def receive(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        acc = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
        gyro = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
        if np.all(np.isfinite(acc)) and np.linalg.norm(acc) >= 1:
            self.publish_acceleration_vector(msg, acc)
        if self.engine.stage in ('complete', 'failed'):
            return
        if self.frame is not None and msg.header.frame_id != self.frame:
            self.engine.fail('IMU frame changed; restart with one consistent input frame.')
            self.save()
            return
        self.frame = msg.header.frame_id
        if self.source and self.source.get('frame_id') and self.source['frame_id'] != self.frame:
            self.engine.fail('Loaded accelerometer frame differs from input IMU frame.')
            self.save()
            return
        if not np.all(np.isfinite(np.r_[stamp, acc, gyro])) or np.linalg.norm(acc) < 1:
            self.engine.gap('Invalid IMU measurement')
            return
        now = time.monotonic()
        self.last_receive = now
        self.last_acceleration = acc
        self.writer.writerow([stamp, *acc, *gyro])
        revision = self.engine.revision
        self.engine.feed(stamp, acc, gyro)
        if self.engine.revision != revision or self.engine.stage in ('complete', 'failed'):
            self.get_logger().info(self.engine.status_line())
            self.save()
            self._ui_snapshot = self._make_ui_snapshot()

    def publish_acceleration_vector(self, imu, acceleration):
        now = time.monotonic()
        if now - self.last_vector_publish < 0.1:
            return
        self.last_vector_publish = now
        marker = Marker()
        marker.header = imu.header
        marker.ns = 'l2_imu_acceleration'
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.points = [Point(x=0.0, y=0.0, z=0.0),
                         Point(x=float(acceleration[0] / self.settings['gravity']),
                               y=float(acceleration[1] / self.settings['gravity']),
                               z=float(acceleration[2] / self.settings['gravity']))]
        marker.scale.x = 0.04
        marker.scale.y = 0.08
        marker.scale.z = 0.12
        marker.color.r = 1.0
        marker.color.g = 0.4
        marker.color.b = 0.05
        marker.color.a = 1.0
        self.vector_publisher.publish(marker)

    def status(self):
        self._handle_selection()
        self._ui_snapshot = self._make_ui_snapshot()
        if self.engine.stage in ('complete', 'failed'):
            return
        now = time.monotonic()
        if now - self.last_log >= 5:
            message = self.engine.status_line()
            if self.last_receive is None or now - self.last_receive > 1:
                message = self.engine.status_line() + ' Waiting for live IMU on ' + self.settings['imu_topic']
            self.get_logger().info(message)
            self.last_log = now
        self.raw_file.flush()

    def save(self):
        report = dict(schema_version=2, frame_id=self.frame, settings=self.settings,
                      imu_topic=self.settings['imu_topic'], gravity_m_s2=self.settings['gravity'],
                      conventions=dict(accelerometer='a_corrected = C_acc @ (a_raw - bias_m_s2)',
                                       gyroscope='omega_corrected = C_gyro @ (omega_raw - bias_rad_s)'),
                      note='Gyro corrected into accelerometer calibration basis; no LiDAR extrinsic determined. '
                           'Constant temperature/bias assumed. No SLAM parameters changed.',
                      **self.engine.report())
        temporary = self.output_path.with_suffix('.tmp')
        temporary.write_text(yaml.safe_dump(report, sort_keys=False))
        temporary.replace(self.output_path)
        self.raw_file.flush()

    def destroy_node(self):
        self._ui_server.shutdown()
        self._ui_server.server_close()
        self.save()
        self.raw_file.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = StaticCalibration()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
