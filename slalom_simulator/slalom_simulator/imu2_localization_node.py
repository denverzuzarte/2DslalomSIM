#!/usr/bin/env python3
"""
IMU2 Localization Node - Dead reckoning using IMU2 data with noise and bias.

This node extends BaseIMULocalizationNode with IMU2-specific parameters.
IMU2 typically has different (often worse) noise characteristics and bias than IMU1.
"""
import rclpy
import numpy as np

from slalom_simulator.base_imu_localization_node import BaseIMULocalizationNode
from slalom_simulator.utils import load_covariance_matrix_3x3


class IMU2LocalizationNode(BaseIMULocalizationNode):
    def __init__(self):
        super().__init__('imu2_localization_node', 'imu2')

    def _declare_imu_parameters(self):
        """Declare IMU2-specific parameters"""
        self.declare_parameters(
            namespace='',
            parameters=[
                ('imu2_R.xx', 0.05),
                ('imu2_R.xy', 0.0),
                ('imu2_R.xt', 0.0),
                ('imu2_R.yy', 0.05),
                ('imu2_R.yt', 0.0),
                ('imu2_R.tt', 0.005),
                ('imu2_bias', [0.05, 0.05, 0.005]),
            ]
        )

    def _load_noise_model(self) -> np.ndarray:
        """Load IMU2 measurement noise (R) matrix"""
        R_dict = {
            'xx': self.get_parameter('imu2_R.xx').value,
            'xy': self.get_parameter('imu2_R.xy').value,
            'xt': self.get_parameter('imu2_R.xt').value,
            'yy': self.get_parameter('imu2_R.yy').value,
            'yt': self.get_parameter('imu2_R.yt').value,
            'tt': self.get_parameter('imu2_R.tt').value,
        }
        return load_covariance_matrix_3x3(R_dict)

    def _load_bias(self) -> np.ndarray:
        """Load IMU2 bias vector"""
        return np.array(self.get_parameter('imu2_bias').value)


def main(args=None):
    rclpy.init(args=args)
    node = IMU2LocalizationNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
