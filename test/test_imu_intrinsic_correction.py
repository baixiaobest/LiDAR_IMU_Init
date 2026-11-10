import numpy as np
import pytest
import yaml

from lidar_imu_init.imu_intrinsic_correction import load_correction, correct_vector, correct_covariance


def test_validated_body_calibration_and_correction(tmp_path):
    report = {'frame_id': 'body', 'stage': 'complete'}
    for name, bias, scale in [('accelerometer', 'bias_m_s2', [1.01, 1.02, 1.03]),
                              ('gyroscope', 'bias_rad_s', [0.99, 0.98, 0.97])]:
        report[name] = {bias: [1.0, 2.0, 3.0], 'correction_matrix': np.diag(scale).tolist(),
                        'validation': {'quality_pass': True}}
    path = tmp_path / 'calibration.yaml'
    path.write_text(yaml.safe_dump(report))
    acc, gyro = load_correction(path)
    np.testing.assert_allclose(correct_vector([2, 3, 4], *acc), [1.01, 1.02, 1.03])
    np.testing.assert_allclose(correct_vector([2, 3, 4], *gyro), [0.99, 0.98, 0.97])
    np.testing.assert_allclose(np.diag(np.asarray(correct_covariance(np.eye(3).flatten(), acc[0])).reshape(3, 3)),
                               [1.01**2, 1.02**2, 1.03**2])
    assert correct_covariance([-1.0] + [0.0] * 8, acc[0])[0] == -1.0
    report['gyroscope']['validation']['quality_pass'] = False
    path.write_text(yaml.safe_dump(report))
    with pytest.raises(ValueError, match='validation'):
        load_correction(path)
