"""Sensor-time calibration state machine, independent of ROS for replay/testing."""
from collections import deque
import numpy as np
from lidar_imu_init.imu_static_calibration import (
    fit_accelerometer, fit_gyroscope, noise_statistics,
    validate_accelerometer, validate_correction, validate_gyroscope,
)

POSE_INSTRUCTIONS = (
    ('Lay flat', 'Support the robot with its body level and feet pointing down. Hold it still through the baseline.'),
    ('Robot head up', 'Pitch the front of the robot upward. Support it and hold still for the pose capture.'),
    ('Robot head down', 'Pitch the front of the robot downward. Support it and hold still.'),
    ('Roll left', 'Roll the robot so its left side is lower than its right side. Hold still.'),
    ('Roll right', 'Roll the robot so its right side is lower than its left side. Hold still.'),
    ('Upside down', 'Carefully support the robot with its feet pointing upward. Hold still.'),
)
GYRO_INSTRUCTIONS = (
    ('Pitch rotation (body Y)', 'Hold still until the start pose is captured. Raise or lower the front/head about the left-to-right body axis, then hold still. About 30–45° of pitch is enough; do not aim for 90°.'),
    ('Roll rotation (body X)', 'After the flat return is confirmed, roll left or right about the front-to-back body axis. Stop at a manageable 30–45° overall tilt and hold still. A pitch motion will not count as roll.'),
    ('Tilted twist (body Z)', 'With the robot supported at about 30–45° tilt, twist it roughly 45–60° about its own feet-to-back body axis, then hold still. A 90° head-up pose is unnecessary. The gravity direction must change by at least 20°.'),
)


class GuidedCalibration:
    def __init__(self, settings, accelerometer=None):
        self.settings = settings
        self.mode = settings['mode']
        if self.mode not in ('accelerometer', 'gyroscope', 'full'):
            raise ValueError('mode must be accelerometer, gyroscope, or full')
        self.accelerometer = accelerometer
        if self.mode == 'gyroscope':
            if accelerometer is None:
                raise ValueError('Gyroscope mode requires accelerometer_calibration')
            validate_correction(accelerometer['correction_matrix'])
            bias = np.asarray(accelerometer['bias_m_s2'])
            if bias.shape != (3,) or not np.all(np.isfinite(bias)):
                raise ValueError('Invalid accelerometer bias')
        self.stage = 'stationary_baseline'
        self.window = deque()
        self.history = deque()
        self.baseline = []
        self.poses = []
        self.acc_validation = []
        self.transitions = []
        self.gyro_validation = []
        self.stationary_gyro_means = []
        self.anchor = None
        self.awaiting_flat = bool(settings.get('body_axis_guidance', False))
        self.gyroscope = None
        self.noise = None
        self.last_stamp = None
        self.next_evaluation = -np.inf
        self.revision = 0
        self.message = 'Hold the robot still for the stationary baseline.'
        self.failure = None

    def gap(self, reason):
        self.window.clear()
        self.history.clear()
        self.baseline.clear()
        self.anchor = None
        if self.settings.get('body_axis_guidance', False) and self.stage == 'gyro_training' and len(self.transitions) < 2:
            self.awaiting_flat = True
        self.message = reason + '; hold still again. The interrupted rotation was discarded.'

    def feed(self, stamp, acc, gyro):
        if self.stage in ('complete', 'failed'):
            return
        # The Go2 LowState clock mapper can clamp a sample to the preceding ROS
        # stamp. Duplicate stamps carry no integration time; ignore them without
        # discarding an otherwise continuous stationary hold or rotation.
        if self.last_stamp is not None and stamp == self.last_stamp:
            return
        if self.last_stamp is not None and (stamp < self.last_stamp or stamp - self.last_stamp > self.settings['max_gap_seconds']):
            self.gap('Timestamp discontinuity')
            self.next_evaluation = stamp
        self.last_stamp = stamp
        row = (float(stamp), np.asarray(acc, dtype=float), np.asarray(gyro, dtype=float))
        self.window.append(row)
        self.history.append(row)
        while self.window and stamp - self.window[0][0] > self.settings['hold_seconds'] + 0.05:
            self.window.popleft()
        # Do not let waiting at one pose grow the recording buffer indefinitely.
        while self.history and stamp - self.history[0][0] > self.settings['max_rotation_seconds']:
            self.history.popleft()
        if stamp < self.next_evaluation:
            return
        self.next_evaluation = stamp + 0.25
        if len(self.window) < 10 or stamp - self.window[0][0] < self.settings['hold_seconds']:
            return
        accs = np.array([r[1] for r in self.window])
        gyros = np.array([r[2] for r in self.window])
        gyro_bias = np.array(self.noise['gyro']['mean']) if self.noise else np.zeros(3)
        midpoint = len(accs) // 2
        acc_std = float(np.max(accs.std(axis=0)))
        acc_shift = float(np.linalg.norm(accs[:midpoint].mean(axis=0) -
                                         accs[midpoint:].mean(axis=0)))
        gyro_std = float(np.max(gyros.std(axis=0)))
        gyro_rate = float(np.linalg.norm(gyros.mean(axis=0) - gyro_bias))
        stationary = (acc_std < self.settings['accel_std_limit'] and
                      acc_shift < self.settings['accel_mean_shift_limit'] and
                      gyro_std < self.settings['gyro_std_limit'] and
                      gyro_rate < self.settings['gyro_norm_limit'])
        if not stationary:
            self.baseline.clear()
            self.message = (f'Stationarity check: accel std {acc_std:.3f}/{self.settings["accel_std_limit"]:.3f} '
                            f'm/s², accel mean shift {acc_shift:.3f}/{self.settings["accel_mean_shift_limit"]:.3f} '
                            f'm/s², gyro std {gyro_std:.4f}/{self.settings["gyro_std_limit"]:.4f} '
                            f'rad/s, gyro rate {gyro_rate:.4f}/{self.settings["gyro_norm_limit"]:.4f} rad/s. '
                            'Hold still for another interval.')
            return
        if self.stage == 'stationary_baseline':
            if not self.baseline:
                self.baseline = list(self.window)
            else:
                self.baseline.extend(r for r in self.window if r[0] > self.baseline[-1][0])
            elapsed = self.baseline[-1][0] - self.baseline[0][0]
            self.message = f'Hold still: baseline {elapsed:.0f}/{self.settings["baseline_seconds"]:.0f} seconds.'
            if elapsed < self.settings['baseline_seconds']:
                return
            self.noise = dict(accelerometer=noise_statistics([r[1] for r in self.baseline]),
                              gyro=noise_statistics([r[2] for r in self.baseline]),
                              measured_rate_hz=float(1 / np.median(np.diff([r[0] for r in self.baseline]))),
                              convention='Per-sample covariance at measured rate, not continuous noise density')
            self.stage = 'gyro_training' if self.mode == 'gyroscope' else 'accel_training'
            self.message = ('Baseline saved. Hold still to capture the first rotation start pose.'
                            if self.mode == 'gyroscope' else
                            f'Baseline saved. Tilt to a new orientation and hold still for {self.settings["hold_seconds"]:g} seconds.')
            self.revision += 1
        mean, gyro_mean = accs.mean(axis=0), gyros.mean(axis=0)
        if self.stage.startswith('accel_'):
            self.accept_acceleration(mean, accs, gyro_mean)
        elif self.stage.startswith('gyro_'):
            self.accept_rotation(mean, gyro_mean, stamp)

    def accept_acceleration(self, mean, accs, gyro_mean):
        previous = self.poses + self.acc_validation
        direction = mean / np.linalg.norm(mean)
        if previous:
            directions = np.array([p['accel_mean_m_s2'] for p in previous])
            directions /= np.linalg.norm(directions, axis=1)[:, None]
            similarities = directions @ direction
            closest_index = int(np.argmax(similarities))
            closest_angle = float(np.degrees(np.arccos(np.clip(similarities[closest_index], -1, 1))))
            if closest_angle < self.settings['min_pose_angle_deg']:
                if closest_index < len(self.poses):
                    closest_name = (POSE_INSTRUCTIONS[closest_index][0]
                                    if closest_index < len(POSE_INSTRUCTIONS)
                                    else f'Training pose {closest_index + 1}')
                else:
                    closest_name = f'Validation pose {closest_index - len(self.poses) + 1}'
                self.message = (f'Closest saved pose: {closest_name} ({closest_angle:.1f}° apart; '
                                f'need {self.settings["min_pose_angle_deg"]:g}°). '
                                + self.tilt_hint(directions, direction))
                return
        pose = dict(accel_mean_m_s2=mean.tolist(), accel_std_m_s2=accs.std(axis=0).tolist(),
                    gyro_mean_rad_s=gyro_mean.tolist(), sample_count=len(accs))
        target = self.poses if self.stage == 'accel_training' else self.acc_validation
        target.append(pose)
        self.stationary_gyro_means.append(gyro_mean.tolist())
        self.revision += 1
        count = self.settings['target_poses'] if self.stage == 'accel_training' else self.settings['validation_count']
        self.message = (f'Pose saved. Next: tilt to a different side or diagonal, then hold still '
                        f'for {self.settings["hold_seconds"]:g} seconds.')
        if len(target) < count:
            return
        if self.stage == 'accel_training':
            try:
                self.accelerometer = fit_accelerometer([p['accel_mean_m_s2'] for p in self.poses], self.settings['gravity'])
                validate_correction(self.accelerometer['correction_matrix'])
            except (ValueError, np.linalg.LinAlgError) as exc:
                self.message = str(exc) + '; collect more varied poses.'
                return
            self.stage = 'accel_validation'
            self.message = 'Accelerometer fit ready. Collect new stationary tilts for independent validation.'
        else:
            result = validate_accelerometer([p['accel_mean_m_s2'] for p in self.acc_validation], self.accelerometer, self.settings['gravity'])
            self.accelerometer['validation'] = result
            if not result['quality_pass']:
                self.fail('Accelerometer held-out validation failed; inspect results and repeat collection.')
            elif self.mode == 'accelerometer':
                self.stage = 'complete'
                self.message = 'Accelerometer calibration and held-out validation passed.'
            else:
                self.stage = 'gyro_training'
                self.anchor = None
                self.awaiting_flat = bool(self.settings.get('body_axis_guidance', False))
                self.message = 'Acceleration validation passed. Hold still, then rotate slowly to a new tilt and stop.'

    def tilt_hint(self, directions, current):
        axes = np.r_[np.eye(3), -np.eye(3)]
        names = ['+X', '+Y', '+Z', '-X', '-Y', '-Z']
        index = int(np.argmin(np.max(directions @ axes.T, axis=0)))
        angle = float(np.degrees(np.arccos(np.clip(current @ axes[index], -1, 1))))
        if angle > 145:
            action = 'Turn the supported robot nearly upside down relative to this pose'
        elif angle > 65:
            action = 'Turn the supported robot onto a different side'
        else:
            action = 'Tilt the supported robot farther in that direction'
        return (f'This pose is too similar to one already saved. {action}, then hold still '
                f'for {self.settings["hold_seconds"]:g} seconds. Aim for acceleration toward sensor {names[index]} '
                f'(about {angle:.0f}° from the current direction). '
                f'Current sensor acceleration XYZ={np.round(current, 2).tolist()}. '
                f'Any new tilt at least {self.settings["min_pose_angle_deg"]:g}° '
                'from all saved poses can be accepted; exact alignment is unnecessary.')

    def progress(self):
        """One baseline plus accepted training/validation samples across enabled phases."""
        validation = self.settings['validation_count']
        phases = [('stationary_baseline', 'Stationary baseline', 1, int(self.noise is not None), 'baseline')]
        if self.mode != 'gyroscope':
            phases += [('accel_training', 'Accelerometer fitting', self.settings['target_poses'], len(self.poses), 'poses'),
                       ('accel_validation', 'Accelerometer validation', validation, len(self.acc_validation), 'poses')]
        if self.mode != 'accelerometer':
            phases += [('gyro_training', 'Gyroscope fitting', self.settings['target_rotations'], len(self.transitions), 'rotations'),
                       ('gyro_validation', 'Gyroscope validation', validation, len(self.gyro_validation), 'rotations')]
        total = sum(p[2] for p in phases)
        completed = sum(min(p[2], p[3]) for p in phases)
        active = next((p for p in phases if p[0] == self.stage), None)
        return dict(completed=completed, total=total,
                    phase=(active[1] if active else self.stage.capitalize()),
                    phase_index=(phases.index(active) + 1 if active else len(phases)),
                    phase_total=len(phases),
                    phase_completed=(min(active[2], active[3]) if active else 0),
                    phase_target=(active[2] if active else 0),
                    unit=(active[4] if active else ''))

    def instruction(self):
        """Physical action to display; acceptance still follows IMU measurements."""
        if self.stage == 'stationary_baseline':
            return dict(title=POSE_INSTRUCTIONS[0][0],
                        detail=POSE_INSTRUCTIONS[0][1] +
                               f' Keep still for {self.settings.get("baseline_seconds", 30.0):g} seconds.')
        if self.stage == 'accel_training':
            index = len(self.poses)
            if index < len(POSE_INSTRUCTIONS):
                title, detail = POSE_INSTRUCTIONS[index]
            else:
                title, detail = ('Extra orientation',
                                 'The six poses did not provide enough sensor-axis coverage. Try a new supported tilt.')
            return dict(title=title, detail=detail +
                        f' Keep still for {self.settings["hold_seconds"]:g} seconds.')
        if self.stage == 'accel_validation':
            return dict(title=f'Diagonal tilt {min(len(self.acc_validation)+1, self.settings["validation_count"])}',
                        detail='Combine pitch and roll in a new direction, then hold still. This pose checks the fit.')
        if self.stage == 'gyro_training':
            index = len(self.transitions)
            if self.awaiting_flat:
                title = ('Hold baseline flat pose' if index == 0 else 'Return to baseline flat pose')
                return dict(title=title,
                            detail=('Place the robot at the same flat orientation used for the initial stationary baseline. '
                                    f'Hold still for {self.settings["hold_seconds"]:g} seconds. '
                                    'This hold and any return motion will not count as a gyro rotation.'))
            title, detail = (GYRO_INSTRUCTIONS[index] if index < len(GYRO_INSTRUCTIONS)
                             else ('Extra rotation', 'Rotate about a different axis while tilted, then hold still.'))
            if self.anchor is None:
                detail = 'First hold still for the rotation start pose. ' + detail
            return dict(title=title, detail=detail +
                        f' The end pose needs {self.settings["hold_seconds"]:g} seconds of stillness.')
        if self.stage == 'gyro_validation':
            return dict(title=f'Check rotation {min(len(self.gyro_validation)+1, self.settings["validation_count"])}',
                        detail='Use a new supported pitch, roll, or tilted body-axis twist. Begin from the previous end pose, rotate slowly, then hold still. This rotation checks the gyro fit.')
        if self.stage == 'complete':
            return dict(title='Calibration complete', detail='The saved result passed its independent checks.')
        return dict(title='Calibration stopped', detail=self.failure or self.message)

    def steps(self):
        """Human-readable steps in the order they count toward progress."""
        items = [('Stationary baseline', POSE_INSTRUCTIONS[0][0])]
        if self.mode != 'gyroscope':
            items += [('Accelerometer fitting', POSE_INSTRUCTIONS[i][0] if i < len(POSE_INSTRUCTIONS)
                       else f'Pose {i+1}') for i in range(self.settings['target_poses'])]
            items += [('Accelerometer validation', f'Diagonal tilt {i+1}')
                      for i in range(self.settings['validation_count'])]
        if self.mode != 'accelerometer':
            items += [('Gyroscope fitting', GYRO_INSTRUCTIONS[i][0] if i < len(GYRO_INSTRUCTIONS)
                       else f'Rotation {i+1}') for i in range(self.settings['target_rotations'])]
            items += [('Gyroscope validation', f'Check rotation {i+1}')
                      for i in range(self.settings['validation_count'])]
        completed = self.progress()['completed']
        result = [dict(phase=phase, title=title, done=i < completed, current=i == completed)
                for i, (phase, title) in enumerate(items)]
        if self.stage == 'gyro_training' and self.awaiting_flat and completed < len(result):
            result[completed]['title'] = ('Hold flat before pitch' if len(self.transitions) == 0
                                          else 'Return flat before roll')
        return result

    def status_line(self):
        p = self.progress()
        phase_count = (f'{p["phase_completed"]}/{p["phase_target"]} {p["unit"]}'
                       if p['phase_target'] else 'finished')
        return (f'[{p["completed"]}/{p["total"]} steps] '
                f'Phase {p["phase_index"]}/{p["phase_total"]}: {p["phase"]} '
                f'({phase_count}). {self.message} '
                f'Current instruction: {self.instruction()["title"]}. {self.instruction()["detail"]}')

    def accept_rotation(self, mean, gyro_mean, stamp):
        direction = np.asarray(self.accelerometer['correction_matrix']) @ (mean - self.accelerometer['bias_m_s2'])
        direction /= np.linalg.norm(direction)
        flat_direction = None
        if self.settings.get('body_axis_guidance', False):
            flat_direction = (np.asarray(self.accelerometer['correction_matrix']) @
                              (np.asarray(self.noise['accelerometer']['mean']) - self.accelerometer['bias_m_s2']))
            flat_direction /= np.linalg.norm(flat_direction)
            if self.stage == 'gyro_training' and self.awaiting_flat:
                flat_angle = float(np.degrees(np.arccos(np.clip(flat_direction @ direction, -1, 1))))
                if flat_angle > 12.0:
                    self.message = f'Return to the baseline flat orientation: currently {flat_angle:.1f}° away; need within 12°. Hold still.'
                    return
                self.awaiting_flat = False
                self.anchor = (stamp, direction)
                self.history = deque([self.history[-1]]) if self.history else deque()
                self.message = 'Baseline flat pose reacquired. Perform the displayed body-axis rotation, then hold still.'
                return
        if self.anchor is None:
            self.anchor = (stamp, direction)
            self.history = deque([self.history[-1]]) if self.history else deque()
            self.message = 'Start pose captured. Perform the displayed body-axis rotation, then hold still; you do not need to return to the original pose.'
            return
        angle = np.degrees(np.arccos(np.clip(self.anchor[1] @ direction, -1, 1)))
        if angle < 20:
            self.message = 'Tilt at least 20 degrees away from the last pose; a vertical spin alone is insufficient.'
            if stamp - self.anchor[0] > self.settings['max_rotation_seconds'] - self.settings['hold_seconds']:
                self.anchor = (stamp, direction)
                self.history.clear()
            return
        if self.stage == 'gyro_training' and flat_direction is not None and len(self.transitions) < 2:
            delta = direction - flat_direction
            index = len(self.transitions)
            primary, other = (0, 1) if index == 0 else (1, 0)
            if abs(delta[primary]) < 1.2 * abs(delta[other]):
                action = 'pitch about body Y' if index == 0 else 'roll about body X'
                self.message = (f'This held pose looks unlike a {action} from baseline flat: '
                                f'body X change {abs(delta[0]):.2f}, Y change {abs(delta[1]):.2f}. '
                                'Return to baseline flat and try the displayed rotation.')
                return
        end = self.window[0][0]
        rows = [r for r in self.history if self.anchor[0] <= r[0] <= end]
        # Include a sample at the anchor time: history stores it separately when anchoring.
        if (len(rows) < 3 or rows[0][0] - self.anchor[0] > self.settings['max_gap_seconds'] or
                stamp - self.anchor[0] > self.settings['max_rotation_seconds']):
            self.anchor = (stamp, direction)
            self.history = deque([self.history[-1]]) if self.history else deque()
            self.message = 'Rotation too long or missing samples. New start pose captured; repeat the rotation.'
            return
        transition = dict(stamps=[r[0] for r in rows], gyro=[r[2].tolist() for r in rows],
                          start_direction=self.anchor[1].tolist(), end_direction=direction.tolist(),
                          start_acceleration_corrected=True)
        target = self.transitions if self.stage == 'gyro_training' else self.gyro_validation
        target.append(transition)
        self.stationary_gyro_means.append(gyro_mean.tolist())
        self.anchor = (stamp, direction)
        self.history = deque([self.history[-1]]) if self.history else deque()
        self.revision += 1
        if self.stage == 'gyro_training' and flat_direction is not None and len(self.transitions) == 1:
            self.awaiting_flat = True
            self.anchor = None
            self.message = 'Pitch saved. Return to the initial flat orientation and hold still before the roll.'
            return
        count = self.settings['target_rotations'] if self.stage == 'gyro_training' else self.settings['validation_count']
        self.message = (f'Rotation saved. Next: rotate about another axis, preferably in the opposite direction, '
                        f'then hold still for {self.settings["hold_seconds"]:g} seconds.')
        if len(target) < count:
            return
        bias = np.array(self.noise['gyro']['mean'])
        if self.stage == 'gyro_training':
            try:
                self.gyroscope = fit_gyroscope(self.transitions, bias)
            except (ValueError, np.linalg.LinAlgError) as exc:
                self.message = str(exc) + '; collect additional varied rotations.'
                return
            self.stage = 'gyro_validation'
            self.message = 'Gyroscope fit ready. Collect new rotations about varied axes for independent validation.'
        else:
            self.gyroscope['validation'] = validate_gyroscope(self.gyro_validation, self.gyroscope['correction_matrix'], bias)
            if self.gyroscope['validation']['quality_pass']:
                self.stage = 'complete'
                self.message = 'Calibration and held-out validation passed. Results saved; Ctrl-C exits.'
            else:
                self.fail('Gyro held-out validation failed; inspect results and repeat collection.')

    def fail(self, reason):
        self.failure, self.stage, self.message = reason, 'failed', reason

    def report(self):
        noise = dict(self.noise) if self.noise else None
        if noise:
            noise = {k: dict(v) if isinstance(v, dict) else v for k, v in noise.items()}
            for key, fit in [('accelerometer', self.accelerometer), ('gyro', self.gyroscope)]:
                if fit:
                    c = np.asarray(fit['correction_matrix'])
                    noise[key]['corrected_covariance'] = (c @ np.asarray(noise[key]['covariance']) @ c.T).tolist()
        return dict(stage=self.stage, mode=self.mode, failure=self.failure,
                    progress=self.progress(),
                    awaiting_flat=self.awaiting_flat,
                    accelerometer=self.accelerometer, gyroscope=self.gyroscope, noise=noise,
                    gyro_stationary_means_rad_s=self.stationary_gyro_means,
                    gyro_bias_variation_std_rad_s=(np.std(self.stationary_gyro_means, axis=0).tolist() if self.stationary_gyro_means else None),
                    accel_training_poses=self.poses, accel_validation_poses=self.acc_validation,
                    gyro_training_transitions=self.transitions, gyro_validation_transitions=self.gyro_validation)
