"""Exercise the actual ROS subscriber, session files, and all three modes."""
import importlib.util
import json
from pathlib import Path
import time
from unittest.mock import patch
from urllib.request import Request, urlopen
import numpy as np
import pytest
import yaml

rclpy = pytest.importorskip('rclpy')
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from visualization_msgs.msg import Marker
from test_imu_static_calibration import run_calibration_session

spec = importlib.util.spec_from_file_location('imu_calibration_node', Path(__file__).parents[1] / 'scripts/imu_static_calibration_node.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def test_rejects_malformed_or_failed_accelerometer_file(tmp_path):
    path = tmp_path / 'bad.yaml'
    path.write_text('[]')
    with pytest.raises(ValueError): module.load_accelerometer(path)
    path.write_text(yaml.safe_dump(dict(accelerometer=dict(validation=dict(quality_pass=False)))))
    with pytest.raises(ValueError): module.load_accelerometer(path)


@pytest.mark.parametrize('mode', ['accelerometer', 'gyroscope', 'full'])
def test_ros_calibration_session(mode, tmp_path):
    source = tmp_path / 'accelerometer.yaml'
    source.write_text(yaml.safe_dump(dict(frame_id='test_imu', fit=dict(
        bias_m_s2=[.2, -.1, .3], correction_matrix=np.diag([.94, 1.06, .98]).tolist(), quality_pass=True))))
    params = dict(mode=mode, accelerometer_calibration=str(source),
                  imu_topic='/test_guided_imu/input',
                  output_dir=str(tmp_path), hold_seconds=.15, baseline_seconds=.5,
                  ui_port=0, open_browser=False)
    rclpy.init()
    node = None
    publisher_node = None
    try:
        node = module.StaticCalibration(parameter_overrides=[Parameter(k, value=v) for k,v in params.items()])
        publisher_node = Node('calibration_synthetic_source')
        publisher = publisher_node.create_publisher(Imu, params['imu_topic'], qos_profile_sensor_data)
        vectors = []
        publisher_node.create_subscription(Marker, '/l2_static_calibration/acceleration_vector',
                                           vectors.append, 10)
        deadline = time.monotonic() + 3
        while publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.01)
        assert publisher.get_subscription_count() > 0
        def send(stamp, acc, gyro):
            message = Imu()
            ns = int(round(stamp * 1e9))
            message.header.stamp.sec, message.header.stamp.nanosec = divmod(ns, 10**9)
            message.header.frame_id = 'test_imu'
            message.linear_acceleration.x, message.linear_acceleration.y, message.linear_acceleration.z = map(float, acc)
            message.angular_velocity.x, message.angular_velocity.y, message.angular_velocity.z = map(float, gyro)
            publisher.publish(message)
            for _ in range(10):
                rclpy.spin_once(node, timeout_sec=.01)
                if node.engine.last_stamp is not None and abs(node.engine.last_stamp - stamp) < 1e-6:
                    break
        send.engine = node.engine
        run_calibration_session(mode, send)
        for _ in range(10):
            rclpy.spin_once(publisher_node, timeout_sec=.01)
        assert vectors and vectors[-1].header.frame_id == 'test_imu'
        assert vectors[-1].type == Marker.ARROW and len(vectors[-1].points) == 2
        assert np.isfinite([vectors[-1].points[1].x, vectors[-1].points[1].y,
                            vectors[-1].points[1].z]).all()
        report = yaml.safe_load(node.output_path.read_text())
        expected = 14 if mode == 'full' else 9 if mode == 'accelerometer' else 6
        assert report['progress']['completed'] == report['progress']['total'] == expected
        if mode != 'gyroscope':
            assert report['accelerometer']['model'] == 'diagonal_bias_scale'
        if mode != 'accelerometer':
            assert report['gyroscope']['model'] == 'diagonal_bias_scale'
        if mode == 'full':
            choices = node._available_accelerometers()
            with patch.object(module, 'load_accelerometer', side_effect=AssertionError('unchanged YAML reparsed')):
                assert node._available_accelerometers() == choices
            with urlopen(node.ui_url + 'api/status', timeout=3) as response:
                ui = json.load(response)
            assert ui['progress']['completed'] == 14
            assert 'Robot head up' in [step['title'] for step in ui['steps']]
            old_path = node.output_path
            selection = json.dumps({'session': old_path.parent.name}).encode()
            request = Request(node.ui_url + 'api/use-accelerometer', data=selection,
                              headers={'Content-Type': 'application/json'}, method='POST')
            with urlopen(request, timeout=3) as response:
                assert response.status == 202
            node.status()
            assert node.engine.mode == 'gyroscope'
            assert node.output_path != old_path and old_path.exists()
            send.engine = node.engine
            run_calibration_session('gyroscope', send)
            report = yaml.safe_load(node.output_path.read_text())
            assert report['progress']['completed'] == report['progress']['total'] == 6
            assert report['gyroscope']['validation']['quality_pass']
    finally:
        if node: node.destroy_node()
        if publisher_node: publisher_node.destroy_node()
        rclpy.shutdown()
