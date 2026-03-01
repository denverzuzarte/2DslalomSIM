#!/usr/bin/env python3
"""
Pressure Sensor Node — simulates a barometric/depth pressure sensor.

Subscribes to /ground_truth (AuvState) and publishes PsData (depth only)
at a configurable frequency with Gaussian noise and zero bias.
PsData.depth = true_z + N(0, sigma_ps^2)
"""
import rclpy
from rclpy.node import Node
import numpy as np

from auv_msgs.msg import AuvState, PsData


class PressureSensorNode(Node):
    def __init__(self):
        super().__init__('pressure_sensor_node')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('ps_freq', 10.0),          # Hz
                ('ps_noise_variance', 0.01), # depth variance (cm^2)
            ]
        )

        self.ps_noise_std = np.sqrt(
            self.get_parameter('ps_noise_variance').value)
        ps_freq = self.get_parameter('ps_freq').value

        # Latest true depth from ground truth
        self.true_z = 0.0

        self.ps_pub = self.create_publisher(PsData, '/pressure_sensor/data', 10)

        self.gt_sub = self.create_subscription(
            AuvState, '/ground_truth', self.gt_callback, 10)

        self.create_timer(1.0 / ps_freq, self.publish_ps)

        self.get_logger().info(
            f'Pressure Sensor node initialized — '
            f'{ps_freq:.1f} Hz, noise std={self.ps_noise_std:.4f} cm, bias=0')

    def gt_callback(self, msg: AuvState):
        """Cache ground truth depth"""
        self.true_z = msg.depth

    def publish_ps(self):
        """Publish noisy depth measurement"""
        noisy_depth = self.true_z + np.random.normal(0.0, self.ps_noise_std)
        msg = PsData()
        msg.depth = float(noisy_depth)
        self.ps_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PressureSensorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
