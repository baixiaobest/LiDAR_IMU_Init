"""Apply a validated diagonal IMU intrinsic calibration in the IMU frame."""
import numpy as np
import yaml


def load_correction(path):
    with open(path, encoding='utf-8') as stream:
        report = yaml.safe_load(stream)
    if not isinstance(report, dict) or report.get('frame_id') != 'body':
        raise ValueError('Expected a Go2 body-frame IMU calibration')
    if report.get('stage') != 'complete':
        raise ValueError('IMU calibration must be complete')
    matrices = []
    biases = []
    for name, units in (('accelerometer', 'bias_m_s2'), ('gyroscope', 'bias_rad_s')):
        fit = report.get(name) or {}
        if fit.get('validation', {}).get('quality_pass') is not True:
            raise ValueError(f'{name} validation did not pass')
        matrix = np.asarray(fit.get('correction_matrix'), dtype=float)
        bias = np.asarray(fit.get(units), dtype=float)
        if matrix.shape != (3, 3) or bias.shape != (3,) or not np.isfinite(matrix).all() or not np.isfinite(bias).all():
            raise ValueError(f'Invalid {name} correction')
        if not np.allclose(matrix, np.diag(np.diag(matrix)), atol=1e-10) or np.any(np.diag(matrix) <= 0):
            raise ValueError(f'{name} must have positive diagonal scale')
        matrices.append(matrix)
        biases.append(bias)
    return (matrices[0], biases[0]), (matrices[1], biases[1])


def correct_vector(values, matrix, bias):
    return matrix @ (np.asarray(values, dtype=float) - bias)


def correct_covariance(values, matrix):
    if values[0] < 0:  # ROS convention: covariance unavailable
        return list(values)
    covariance = np.asarray(values, dtype=float).reshape(3, 3)
    return (matrix @ covariance @ matrix.T).reshape(9).tolist()
