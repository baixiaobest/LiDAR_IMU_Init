#!/usr/bin/env python3
"""Publish calibrated Go2 body IMU measurements for LiDAR_IMU_Init."""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

from lidar_imu_init.imu_intrinsic_correction import (load_correction, correct_vector,
                                                     correct_covariance)


class ImuIntrinsicCorrection(Node):
    def __init__(self):
        super().__init__('go2_body_imu_intrinsic_correction')
        path = self.declare_parameter('calibration_file', '').value
        input_topic = self.declare_parameter('input_topic', '/li_init_go2/input/imu').value
        output_topic = self.declare_parameter('output_topic', '/li_init_go2/input/imu_corrected').value
        self.acc, self.gyro = load_correction(path)
        self.publisher = self.create_publisher(Imu, output_topic, 50)
        self.subscription = self.create_subscription(Imu, input_topic, self.on_imu, qos_profile_sensor_data)
        self.get_logger().info(f'Applying body IMU intrinsics from {path}: {input_topic} -> {output_topic}')

    def on_imu(self, message):
        if message.header.frame_id != 'body':
            self.get_logger().error(f'Expected body IMU frame, got {message.header.frame_id!r}', throttle_duration_sec=5.0)
            return
        acc = np.array([message.linear_acceleration.x, message.linear_acceleration.y, message.linear_acceleration.z])
        gyro = np.array([message.angular_velocity.x, message.angular_velocity.y, message.angular_velocity.z])
        if not np.isfinite(acc).all() or not np.isfinite(gyro).all():
            self.get_logger().error('Skipping nonfinite IMU sample', throttle_duration_sec=5.0)
            return
        corrected = Imu()
        corrected.header = message.header
        corrected.orientation = message.orientation
        corrected.orientation_covariance = message.orientation_covariance
        corrected_acc = correct_vector(acc, *self.acc)
        corrected_gyro = correct_vector(gyro, *self.gyro)
        corrected.linear_acceleration.x, corrected.linear_acceleration.y, corrected.linear_acceleration.z = corrected_acc.tolist()
        corrected.angular_velocity.x, corrected.angular_velocity.y, corrected.angular_velocity.z = corrected_gyro.tolist()
        corrected.linear_acceleration_covariance = correct_covariance(message.linear_acceleration_covariance, self.acc[0])
        corrected.angular_velocity_covariance = correct_covariance(message.angular_velocity_covariance, self.gyro[0])
        self.publisher.publish(corrected)


def main():
    rclpy.init()
    node = ImuIntrinsicCorrection()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
