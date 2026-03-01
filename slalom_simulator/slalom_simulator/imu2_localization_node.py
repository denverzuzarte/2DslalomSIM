#!/usr/bin/env python3
"""
IMU2 Localization Node — 3D dead reckoning, higher noise sensor (5x IMU1).
"""
import rclpy
import numpy as np

from slalom_simulator.base_imu_localization_node import BaseIMULocalizationNode
from slalom_simulator.utils import load_covariance_matrix_4x4


class IMU2LocalizationNode(BaseIMULocalizationNode):
    def __init__(self):
        super().__init__('imu2_localization_node', 'imu2')

    def _declare_imu_parameters(self):
        self.declare_parameters(
            namespace='',
            parameters=[
                ('imu2_R.xx', 0.05),
                ('imu2_R.yy', 0.05),
                ('imu2_R.zz', 0.025),
                ('imu2_R.tt', 0.005),
                # bias: [ax, ay, az, alpha]
                ('imu2_bias', [0.05, 0.05, 0.025, 0.005]),
            ]
        )

    def _load_noise_model(self) -> np.ndarray:
        R_dict = {
            'xx': self.get_parameter('imu2_R.xx').value,
            'yy': self.get_parameter('imu2_R.yy').value,
            'zz': self.get_parameter('imu2_R.zz').value,
            'tt': self.get_parameter('imu2_R.tt').value,
        }
        return load_covariance_matrix_4x4(R_dict)

    def _load_bias(self) -> np.ndarray:
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
