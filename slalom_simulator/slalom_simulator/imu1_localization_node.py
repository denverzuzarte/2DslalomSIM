#!/usr/bin/env python3
"""
IMU1 Localization Node - Dead reckoning using IMU1 data with noise and bias.

This node extends BaseIMULocalizationNode with IMU1-specific parameters.
"""
import rclpy
import numpy as np

from slalom_simulator.base_imu_localization_node import BaseIMULocalizationNode
from slalom_simulator.utils import load_covariance_matrix_3x3


class IMU1LocalizationNode(BaseIMULocalizationNode):
    def __init__(self):
        super().__init__('imu1_localization_node', 'imu1')

    def _declare_imu_parameters(self):
        """Declare IMU1-specific parameters"""
        self.declare_parameters(
            namespace='',
            parameters=[
                ('imu1_R.xx', 0.01),
                ('imu1_R.xy', 0.0),
                ('imu1_R.xt', 0.0),
                ('imu1_R.yy', 0.01),
                ('imu1_R.yt', 0.0),
                ('imu1_R.tt', 0.001),
                ('imu1_bias', [0.01, 0.01, 0.001]),
            ]
        )

    def _load_noise_model(self) -> np.ndarray:
        """Load IMU1 measurement noise (R) matrix"""
        R_dict = {
            'xx': self.get_parameter('imu1_R.xx').value,
            'xy': self.get_parameter('imu1_R.xy').value,
            'xt': self.get_parameter('imu1_R.xt').value,
            'yy': self.get_parameter('imu1_R.yy').value,
            'yt': self.get_parameter('imu1_R.yt').value,
            'tt': self.get_parameter('imu1_R.tt').value,
        }
        return load_covariance_matrix_3x3(R_dict)

    def _load_bias(self) -> np.ndarray:
        """Load IMU1 bias vector"""
        return np.array(self.get_parameter('imu1_bias').value)


def main(args=None):
    rclpy.init(args=args)
    node = IMU1LocalizationNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
