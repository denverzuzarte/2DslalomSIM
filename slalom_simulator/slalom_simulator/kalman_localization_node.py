#!/usr/bin/env python3
"""
Kalman Localization Node — PLACEHOLDER/STUB
Fuses IMU1, IMU2, and pressure sensor (PS) state estimates;
publishes to /kalman/state as AuvState.

Publishing is DISABLED by default (kalman_enabled param = false).
Set kalman_enabled:=true in params or launch to enable output.

Z measurement: PS depth is used as the primary Z observation.
Vz: differentiated from consecutive PS readings.

TODO: Implement proper UKF with 8D state [x,y,z,yaw,vx,vy,vz,wyaw].
"""
import rclpy
from rclpy.node import Node
import numpy as np

from auv_msgs.msg import AuvState, PsData
from slalom_simulator.utils import normalize_angle


class KalmanLocalizationNode(Node):
    def __init__(self):
        super().__init__('kalman_localization_node')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('vehicle_start_x', 400.0),
                ('vehicle_start_y', 500.0),
                ('vehicle_start_z', 0.0),
                ('kalman_enabled', False),   # off by default
                ('ps_weight_z', 0.7),
            ]
        )

        start_x = self.get_parameter('vehicle_start_x').value
        start_y = self.get_parameter('vehicle_start_y').value
        start_z = self.get_parameter('vehicle_start_z').value
        self.kalman_enabled = self.get_parameter('kalman_enabled').value
        self.ps_weight_z = self.get_parameter('ps_weight_z').value

        # 8D state: [x, y, z, yaw, vx, vy, vz, wyaw]
        self.state = np.array([start_x, start_y, start_z,
                               0.0, 0.0, 0.0, 0.0, 0.0])
        self.P = np.eye(8) * 0.1

        # Latest AuvState from each IMU
        self.imu1_data: AuvState = None
        self.imu2_data: AuvState = None

        # PS state for Z fusion
        self.ps_z: float = None
        self.ps_z_prev: float = None
        self.ps_time_prev: float = None

        # Publisher
        self.state_pub = self.create_publisher(AuvState, '/kalman/state', 10)

        # Subscribers — listen to IMU state estimates
        self.imu1_sub = self.create_subscription(
            AuvState, '/imu1/state', self.imu1_callback, 10)
        self.imu2_sub = self.create_subscription(
            AuvState, '/imu2/state', self.imu2_callback, 10)
        # Pressure sensor for Z fusion
        self.ps_sub = self.create_subscription(
            PsData, '/pressure_sensor/data', self.ps_callback, 10)

        if self.kalman_enabled:
            self.get_logger().info('Kalman node initialized — publishing ENABLED (STUB, naive average)')
            self.get_logger().warn('Currently naive average of IMU1/IMU2 + PS-Z. Replace with UKF!')
        else:
            self.get_logger().info('Kalman node initialized — publishing DISABLED (kalman_enabled=false)')

    def ps_callback(self, msg: PsData):
        """Cache PS depth and compute Vz by differentiation."""
        now = self.get_clock().now().nanoseconds * 1e-9
        new_z = msg.depth

        if self.ps_z is not None and self.ps_time_prev is not None:
            dt_ps = now - self.ps_time_prev
            if 0 < dt_ps < 0.5:
                ps_vz = (new_z - self.ps_z) / dt_ps
                w = self.ps_weight_z
                self.state[6] = (1.0 - w) * self.state[6] + w * ps_vz

        self.ps_z_prev = self.ps_z
        self.ps_z = new_z
        self.ps_time_prev = now

    def imu1_callback(self, msg: AuvState):
        self.imu1_data = msg
        self._fuse()

    def imu2_callback(self, msg: AuvState):
        self.imu2_data = msg
        self._fuse()

    def _fuse(self):
        """
        PLACEHOLDER: naive average of IMU1 and IMU2 state estimates,
        with PS used as primary Z measurement.
        Replace with UKF prediction + update steps.
        """
        if self.imu1_data is None or self.imu2_data is None:
            return

        d1 = self.imu1_data
        d2 = self.imu2_data

        # Simple average for XY/yaw/velocities
        x   = (d1.position.x + d2.position.x) / 2.0
        y   = (d1.position.y + d2.position.y) / 2.0
        yaw = normalize_angle(
            (d1.orientation.yaw + d2.orientation.yaw) / 2.0)
        vx  = (d1.velocity.x + d2.velocity.x) / 2.0
        vy  = (d1.velocity.y + d2.velocity.y) / 2.0
        wyaw = (d1.angular_velocity.z + d2.angular_velocity.z) / 2.0

        # Z: PS is primary; fall back to IMU average if PS unavailable
        imu_z  = (d1.position.z + d2.position.z) / 2.0
        imu_vz = (d1.velocity.z + d2.velocity.z) / 2.0
        if self.ps_z is not None:
            w = self.ps_weight_z
            z  = (1.0 - w) * imu_z + w * self.ps_z
            # Vz from self.state[6] is already blended in ps_callback
            vz = self.state[6]
        else:
            z  = imu_z
            vz = imu_vz

        self.state = np.array([x, y, z, yaw, vx, vy, vz, wyaw])

        if not self.kalman_enabled:
            return

        stamp = d1.header.stamp
        self._publish(stamp)

    def _publish(self, stamp):
        x, y, z, yaw, vx, vy, vz, wyaw = self.state
        msg = AuvState()
        msg.header.stamp = stamp
        msg.header.frame_id = 'world'
        msg.position.x = float(x)
        msg.position.y = float(y)
        msg.position.z = float(z)
        msg.velocity.x = float(vx)
        msg.velocity.y = float(vy)
        msg.velocity.z = float(vz)
        msg.orientation.roll = 0.0
        msg.orientation.pitch = 0.0
        msg.orientation.yaw = float(yaw)
        msg.angular_velocity.x = 0.0
        msg.angular_velocity.y = 0.0
        msg.angular_velocity.z = float(wyaw)
        msg.depth = float(z)
        self.state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = KalmanLocalizationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
