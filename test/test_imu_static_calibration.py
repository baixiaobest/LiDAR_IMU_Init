import numpy as np
import pytest
from collections import deque


def test_noise_and_physical_matrix_checks():
    from lidar_imu_init.imu_static_calibration import noise_statistics, validate_correction
    rng = np.random.default_rng(9)
    noise = rng.normal(size=(10000, 3)) * [0.01, .02, .03] + [.1, -.2, .3]
    stats = noise_statistics(noise)
    np.testing.assert_allclose(stats['mean'], [.1, -.2, .3], atol=.001)
    np.testing.assert_allclose(np.diag(stats['covariance']), np.array([.01, .02, .03])**2, rtol=.06)
    with pytest.raises(ValueError):
        validate_correction(np.diag([1, 1, -1]))


def test_gyro_only_requires_accelerometer_and_gap_discards_rotation():
    from lidar_imu_init.guided_imu_calibration import GuidedCalibration
    with pytest.raises(ValueError, match='requires'):
        GuidedCalibration(dict(mode='gyroscope'))
    engine = run_calibration_session('gyroscope')
    engine.stage = 'gyro_training'; engine.anchor = (1., np.array([0, 0, 1.]))
    engine.gap('test')
    assert engine.anchor is None and not engine.history


def test_duplicate_timestamp_does_not_restart_stationary_baseline():
    from lidar_imu_init.guided_imu_calibration import GuidedCalibration
    settings = dict(mode='accelerometer', hold_seconds=.15, baseline_seconds=.5,
                    accel_std_limit=.35, accel_mean_shift_limit=.25,
                    gyro_std_limit=.02, gyro_norm_limit=.12, max_gap_seconds=.1,
                    max_rotation_seconds=60., gravity=9.81, min_pose_angle_deg=15.,
                    target_poses=6, target_rotations=3, validation_count=2)
    engine = GuidedCalibration(settings)
    for index in range(100):
        stamp = 100 + index * .01
        engine.feed(stamp, [0, 0, 9.81], [0, 0, 0])
        if index % 7 == 0:
            engine.feed(stamp, [0, 0, 9.81], [0, 0, 0])
    assert engine.noise is not None
    assert len(engine.poses) == 1


def test_body_gyro_return_to_flat_does_not_count_as_roll():
    from lidar_imu_init.guided_imu_calibration import GuidedCalibration
    settings = dict(mode='gyroscope', body_axis_guidance=True, hold_seconds=.15,
                    max_gap_seconds=.1, max_rotation_seconds=60., target_rotations=3,
                    validation_count=2)
    fit = dict(bias_m_s2=[0., 0., 0.], correction_matrix=np.eye(3).tolist())
    engine = GuidedCalibration(settings, fit)
    engine.stage = 'gyro_training'
    engine.noise = dict(accelerometer=dict(mean=[0., 0., 9.81]),
                        gyro=dict(mean=[0., 0., 0.]))

    def segment(start, end, axis):
        rows = [(float(t), np.zeros(3), np.eye(3)[axis])
                for t in np.arange(start, end + .001, .01)]
        engine.history = deque(rows)
        engine.window = deque(rows[-16:])

    engine.accept_rotation(np.array([0., 0., 9.81]), np.zeros(3), 0.)
    assert not engine.awaiting_flat
    segment(0., 1., 1)
    engine.accept_rotation(np.array([9.81, 0., 0.]), np.zeros(3), 1.)
    assert len(engine.transitions) == 1 and engine.awaiting_flat

    engine.accept_rotation(np.array([0., 0., 9.81]), np.zeros(3), 2.)
    assert len(engine.transitions) == 1 and not engine.awaiting_flat
    assert engine.anchor[0] == 2.
    segment(2., 3., 1)
    engine.accept_rotation(np.array([9.81, 0., 0.]), np.zeros(3), 3.)
    assert len(engine.transitions) == 1
    assert 'unlike a roll' in engine.message

    segment(2., 4., 0)
    engine.accept_rotation(np.array([0., 9.81, 0.]), np.zeros(3), 4.)
    assert len(engine.transitions) == 2


def test_stationarity_accepts_l2_vibration_but_rejects_slow_tilt():
    from lidar_imu_init.guided_imu_calibration import GuidedCalibration
    settings = dict(mode='accelerometer', hold_seconds=3., baseline_seconds=4.,
                    accel_std_limit=.35, accel_mean_shift_limit=.25,
                    gyro_std_limit=.02, gyro_norm_limit=.12, max_gap_seconds=.1,
                    max_rotation_seconds=60., gravity=9.81, min_pose_angle_deg=25.,
                    target_poses=6, target_rotations=3, validation_count=2)
    rng = np.random.default_rng(14)
    engine = GuidedCalibration(settings)
    for index in range(2000):
        acceleration = [3.27, 0, 9.8] + rng.normal(scale=[.22, .14, .23])
        engine.feed(100 + index*.004, acceleration, rng.normal(scale=.005, size=3))
    assert engine.stage == 'accel_training'
    for index in range(1000):
        angle = np.radians(6 * index / 1000)
        acceleration = [9.81*np.sin(angle), 0, 9.81*np.cos(angle)]
        engine.feed(108 + index*.004, acceleration, [0, 0, 0])
    assert 'accel mean shift' in engine.message


def test_tilt_hint_explains_opposite_direction_as_turning_over():
    from lidar_imu_init.guided_imu_calibration import GuidedCalibration
    settings = dict(mode='full', hold_seconds=3., min_pose_angle_deg=25.,
                    target_poses=6, target_rotations=3, validation_count=2)
    engine = GuidedCalibration(settings)
    current = np.array([-.24, 0., .97]); current /= np.linalg.norm(current)
    message = engine.tilt_hint(np.array([[0., 0., 1.]]), current)
    assert 'upside down' in message
    assert 'exact alignment is unnecessary' in message
    assert engine.progress()['total'] == 14
    assert 'Phase 1/5: Stationary baseline' in engine.status_line()


def test_six_pose_instructions_follow_physical_order():
    from lidar_imu_init.guided_imu_calibration import GuidedCalibration
    settings = dict(mode='full', hold_seconds=3., baseline_seconds=30.,
                    target_poses=6, target_rotations=3, validation_count=2)
    engine = GuidedCalibration(settings)
    assert engine.instruction()['title'] == 'Lay flat'
    engine.stage = 'accel_training'
    expected = ['Lay flat', 'Robot head up', 'Robot head down',
                'Roll left', 'Roll right', 'Upside down']
    for count, name in enumerate(expected):
        engine.poses = [{} for _ in range(count)]
        assert engine.instruction()['title'] == name
        assert engine.steps()[1 + count]['title'] == name
    engine.poses = [{} for _ in range(6)]
    assert engine.instruction()['title'] == 'Extra orientation'


def test_similar_pose_reports_closest_saved_pose_and_angle():
    from lidar_imu_init.guided_imu_calibration import GuidedCalibration
    settings = dict(mode='accelerometer', hold_seconds=3., min_pose_angle_deg=15.,
                    target_poses=6, target_rotations=3, validation_count=2)
    engine = GuidedCalibration(settings)
    engine.stage = 'accel_training'
    engine.poses = [dict(accel_mean_m_s2=[0, 0, 9.81]),
                    dict(accel_mean_m_s2=[9.81, 0, 0])]
    direction = np.array([np.cos(np.radians(10)), np.sin(np.radians(10)), 0.])
    engine.accept_acceleration(direction * 9.81, np.tile(direction * 9.81, (10, 1)), np.zeros(3))
    assert len(engine.poses) == 2
    assert 'Robot head up (10.0° apart; need 15°)' in engine.message


def run_calibration_session(mode, send=None):
    from scipy.spatial.transform import Rotation
    from lidar_imu_init.guided_imu_calibration import GuidedCalibration
    settings = dict(mode=mode, hold_seconds=.15,
                    baseline_seconds=.5, accel_std_limit=.35,
                    accel_mean_shift_limit=.25, gyro_std_limit=.02,
                    gyro_norm_limit=.12, min_pose_angle_deg=25., target_poses=6,
                    target_rotations=3, validation_count=2, gravity=9.81,
                    max_gap_seconds=.1, max_rotation_seconds=60.)
    known = dict(bias_m_s2=[.2, -.1, .3], correction_matrix=np.diag([.94, 1.06, .98]).tolist())
    if mode == 'gyroscope':
        engine = GuidedCalibration(settings, known)
    else:
        engine = GuidedCalibration(settings)
    if send:
        engine = send.engine
    stamp = 100.
    gyro_bias = np.array([.012, -.017, .009])
    acc_bias = np.array(known['bias_m_s2'])
    acc_matrix = np.array(known['correction_matrix'])
    gyro_matrix = np.diag([.95, 1.04, .97])
    def feed(direction, corrected_gyro=None):
        nonlocal stamp
        stamp += .01
        acceleration = np.linalg.solve(acc_matrix, np.asarray(direction)*9.81) + acc_bias
        angular_rate = (np.linalg.solve(gyro_matrix, corrected_gyro) + gyro_bias
                        if corrected_gyro is not None else gyro_bias)
        if send:
            send(stamp, acceleration, angular_rate)
        else:
            engine.feed(stamp, acceleration, angular_rate)
    def hold(direction, count=50):
        for _ in range(count): feed(direction)
    direction = np.array([0., 0., 1.])
    hold(direction, 100)
    if mode != 'gyroscope':
        for next_direction in ([0, 0, -1], [1, 0, 0], [-1, 0, 0],
                               [0, 1, 0], [0, -1, 0]):
            direction = np.array(next_direction, dtype=float)
            hold(direction)
        assert engine.stage == 'accel_validation', engine.message
        for next_direction in ([1, 1, 1], [-1, -1, -1]):
            direction = np.array(next_direction, dtype=float) / np.sqrt(3)
            hold(direction)
        assert engine.stage == ('complete' if mode == 'accelerometer' else 'gyro_training'), engine.message
        np.testing.assert_allclose(engine.accelerometer['correction_matrix'], acc_matrix, atol=.01)
    if mode != 'accelerometer':
        hold(direction)
        vectors = ([.8, .5, -.1], [-.2, .7, .9], [-.8, -.3, .7],
                   [.5, -.9, .3], [.7, .2, .8])
        for vector in vectors:
            omega = np.array(vector) / np.linalg.norm(vector)
            omega *= 1.2
            if np.degrees(np.arccos(np.clip(direction @ Rotation.from_rotvec(-omega).apply(direction),-1,1))) < 20:
                omega = np.roll(omega, 1)
            for _ in range(100):
                direction = Rotation.from_rotvec(-omega*.01).apply(direction)
                feed(direction, omega)
            hold(direction)
        assert engine.stage == 'complete', (engine.stage, engine.message)
        np.testing.assert_allclose(engine.gyroscope['correction_matrix'], gyro_matrix, atol=.03)
    progress = engine.progress()
    total = 14 if mode == 'full' else 9 if mode == 'accelerometer' else 6
    assert progress['completed'] == progress['total'] == total
    return engine


@pytest.mark.parametrize('mode', ['accelerometer', 'gyroscope', 'full'])
def test_six_faces_and_three_varied_rotations(mode):
    run_calibration_session(mode)
