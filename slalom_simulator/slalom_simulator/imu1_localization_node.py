#!/usr/bin/env python3
"""
IMU1 Localization Node — 3D dead reckoning, lower noise sensor.
"""
import rclpy
import numpy as np

from slalom_simulator.base_imu_localization_node import BaseIMULocalizationNode
from slalom_simulator.utils import load_covariance_matrix_4x4


class IMU1LocalizationNode(BaseIMULocalizationNode):
    def __init__(self):
        super().__init__('imu1_localization_node', 'imu1')

    def _declare_imu_parameters(self):
        self.declare_parameters(
            namespace='',
            parameters=[
                ('imu1_R.xx', 0.01),
                ('imu1_R.yy', 0.01),
                ('imu1_R.zz', 0.005),
                ('imu1_R.tt', 0.001),
                # bias: [ax, ay, az, alpha]
                ('imu1_bias', [0.01, 0.01, 0.005, 0.001]),
            ]
        )

    def _load_noise_model(self) -> np.ndarray:
        R_dict = {
            'xx': self.get_parameter('imu1_R.xx').value,
            'yy': self.get_parameter('imu1_R.yy').value,
            'zz': self.get_parameter('imu1_R.zz').value,
            'tt': self.get_parameter('imu1_R.tt').value,
        }
        return load_covariance_matrix_4x4(R_dict)

    def _load_bias(self) -> np.ndarray:
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
