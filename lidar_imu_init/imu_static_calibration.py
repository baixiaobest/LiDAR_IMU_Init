"""Six-pose accelerometer and three-axis gyro calibration (SI units)."""
import numpy as np


def fit_accelerometer(means, gravity=9.81):
    """Six-pose fit: bias and scale on each axis, without cross-axis terms."""
    from scipy.optimize import least_squares
    samples = np.asarray(means, dtype=float)
    if samples.ndim != 2 or samples.shape[1] != 3 or len(samples) < 6:
        raise ValueError('Accelerometer fit needs at least six stationary orientations')
    if not np.all(np.isfinite(samples)):
        raise ValueError('Accelerometer samples must be finite')
    bias_guess = np.clip(0.5 * (samples.max(axis=0) + samples.min(axis=0)), -4.9, 4.9)
    def residual(x):
        return np.linalg.norm((samples - x[:3]) * x[3:], axis=1) - gravity
    result = least_squares(residual, np.r_[bias_guess, np.ones(3)],
                           bounds=(np.r_[-5*np.ones(3), .5*np.ones(3)],
                                   np.r_[5*np.ones(3), 2*np.ones(3)]), max_nfev=300)
    singular = np.linalg.svd(result.jac, compute_uv=False)
    if not result.success or singular[-1] < 1e-8 or singular[0] / singular[-1] > 1000:
        raise ValueError('Six poses do not constrain per-axis bias and scale; use opposite IMU-axis orientations')
    bias, scales = result.x[:3], result.x[3:]
    corrected = (samples - bias) * scales
    directions = corrected / np.linalg.norm(corrected, axis=1)[:, None]
    coverage = np.linalg.eigvalsh(directions.T @ directions / len(directions))
    if coverage[0] < .12 or np.any(directions.min(axis=0) > -.35) or np.any(directions.max(axis=0) < .35):
        raise ValueError('Fit needs gravity on both sides of all three IMU axes')
    errors = np.linalg.norm(corrected, axis=1) - gravity
    return dict(model='diagonal_bias_scale', bias_m_s2=bias.tolist(),
                correction_matrix=np.diag(scales).tolist(), principal_scale_factors=scales.tolist(),
                corrected_norm_rms_m_s2=float(np.sqrt(np.mean(errors**2))),
                corrected_norm_max_error_m_s2=float(np.max(np.abs(errors))),
                coverage_eigenvalues=coverage.tolist(),
                design_condition_number=float(singular[0] / singular[-1]))


def validate_correction(matrix):
    matrix = np.asarray(matrix, dtype=float)
    if matrix.shape != (3, 3) or not np.all(np.isfinite(matrix)):
        raise ValueError('Correction matrix must be finite and 3x3')
    scales = np.linalg.svd(matrix, compute_uv=False)
    if np.linalg.det(matrix) <= 0 or scales.min() < 0.5 or scales.max() > 2.0:
        raise ValueError('Correction matrix is outside physical bounds (scale 0.5 to 2)')
    return matrix


def validate_accelerometer(means, calibration, gravity=9.81):
    corrected = (np.asarray(means) - calibration['bias_m_s2']) @ np.asarray(calibration['correction_matrix']).T
    errors = np.linalg.norm(corrected, axis=1) - gravity
    rms, maximum = float(np.sqrt(np.mean(errors**2))), float(np.max(np.abs(errors)))
    return dict(norm_rms_m_s2=rms, norm_max_error_m_s2=maximum,
                quality_pass=rms < 0.15 and maximum < 0.4)


def integrate_gravity(stamps, raw_gyro, direction, matrix, bias, max_gap=0.1):
    """Transport a fixed world vector in body coordinates: u_next=Exp(-omega dt)u."""
    from scipy.spatial.transform import Rotation
    stamps, rates = np.asarray(stamps), np.asarray(raw_gyro)
    dt = np.diff(stamps)
    if len(dt) < 1 or np.any(dt <= 0) or np.any(dt > max_gap):
        raise ValueError('Non-increasing timestamps or missing IMU samples in rotation')
    midpoint = (0.5 * (rates[:-1] + rates[1:]) - bias) @ np.asarray(matrix).T
    rotations = Rotation.from_rotvec(-midpoint * dt[:, None]).as_matrix()
    result = np.array(direction, dtype=float)
    for rotation in rotations:
        result = rotation @ result
    return result / np.linalg.norm(result)


def gyro_errors(transitions, matrix, bias):
    residuals = []
    for transition in transitions:
        predicted = integrate_gravity(transition['stamps'], transition['gyro'],
                                      transition['start_direction'], matrix, bias)
        residuals.append(predicted - transition['end_direction'])
    return np.asarray(residuals).ravel()


def fit_gyroscope(transitions, bias):
    """Fit of three gyro axis scales using gravity changes during rotations."""
    from scipy.optimize import least_squares
    if len(transitions) < 3:
        raise ValueError('Gyro fit needs at least three varied rotations')
    result = least_squares(lambda scales: gyro_errors(transitions, np.diag(scales), bias),
                           np.ones(3), bounds=(.5*np.ones(3), 2*np.ones(3)), max_nfev=150)
    singular = np.linalg.svd(result.jac, compute_uv=False)
    if not result.success or singular[-1] <= 1e-8 or singular[0] / singular[-1] > 1000:
        raise ValueError('Gyro scales are unconstrained; rotate about three varied axes while tilted')
    matrix = validate_correction(np.diag(result.x))
    return dict(model='diagonal_bias_scale', bias_rad_s=np.asarray(bias).tolist(),
                correction_matrix=matrix.tolist(),
                jacobian_condition_number=float(singular[0] / singular[-1]),
                training=validate_gyroscope(transitions, matrix, bias))


def validate_gyroscope(transitions, matrix, bias):
    angles = []
    for t in transitions:
        predicted = integrate_gravity(t['stamps'], t['gyro'], t['start_direction'], matrix, bias)
        # atan2 is stable for very small errors.
        end = np.asarray(t['end_direction'])
        angles.append(float(np.degrees(np.arctan2(np.linalg.norm(np.cross(predicted, end)), predicted @ end))))
    rms, maximum = float(np.sqrt(np.mean(np.square(angles)))), float(max(angles))
    return dict(direction_errors_deg=angles, direction_rms_deg=rms,
                direction_max_error_deg=maximum, quality_pass=rms < 2 and maximum < 5)


def noise_statistics(samples):
    """Per-sample covariance and lag-one correlation; never a noise density."""
    samples = np.asarray(samples)
    centered = samples - samples.mean(axis=0)
    denom = np.sqrt(np.sum(centered[:-1]**2, axis=0) * np.sum(centered[1:]**2, axis=0))
    lag = np.divide(np.sum(centered[:-1] * centered[1:], axis=0), denom,
                    out=np.zeros(samples.shape[1]), where=denom > 0)
    return dict(mean=samples.mean(axis=0).tolist(), covariance=np.cov(samples, rowvar=False).tolist(),
                lag_one_correlation=lag.tolist(), sample_count=len(samples))
