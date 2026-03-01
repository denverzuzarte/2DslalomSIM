#!/usr/bin/env python3
"""
Base IMU Localization Node — Abstract base for 3D IMU dead-reckoning.

Subscribes to clean combined acceleration from /imuN/accel (ImuAccel msg),
adds measurement noise (4x4 R matrix) + bias, then double-integrates
to estimate 3D AUV state.

Z-axis fusion with pressure sensor (/pressure_sensor/data, PsData):
  - PS depth is used as the PRIMARY measurement for Z (replaces IMU Z by default).
  - Vz is estimated by differentiating consecutive PS readings (numerical diff).
  - ps_weight_z (param, default 0.7) blends PS and IMU-integrated Z.
    0 = pure IMU, 1 = pure PS.

State: [x, y, z, yaw, vx, vy, vz, wyaw]
"""
import rclpy
from rclpy.node import Node
import numpy as np
from abc import ABC, abstractmethod

from auv_msgs.msg import AuvState, ImuAccel, PsData
from slalom_simulator.utils import load_covariance_matrix_4x4, normalize_angle


class BaseIMULocalizationNode(Node, ABC):
    """Abstract base class for 3D IMU localization nodes with PS Z-fusion."""

    def __init__(self, node_name: str, imu_name: str):
        super().__init__(node_name)
        self.imu_name = imu_name

        self.declare_parameters(
            namespace='',
            parameters=[
                ('vehicle_start_x', 400.0),
                ('vehicle_start_y', 500.0),
                ('vehicle_start_z', 0.0),
                ('ps_weight_z', 0.7),   # PS weight in Z fusion (0=IMU-only, 1=PS-only)
            ]
        )
        self._declare_imu_parameters()

        # 3D state: [x, y, z, yaw, vx, vy, vz, wyaw]
        start_x = self.get_parameter('vehicle_start_x').value
        start_y = self.get_parameter('vehicle_start_y').value
        start_z = self.get_parameter('vehicle_start_z').value
        self.state = np.array([start_x, start_y, start_z,
                               0.0, 0.0, 0.0, 0.0, 0.0])

        self.R = self._load_noise_model()   # 4x4 covariance [ax, ay, az, alpha]
        self.bias = self._load_bias()       # 4-element [ax_b, ay_b, az_b, alpha_b]

        self.ps_weight_z = self.get_parameter('ps_weight_z').value

        self.last_time = None

        # Pressure sensor state for Z fusion
        self.ps_z: float = None           # latest PS depth reading
        self.ps_z_prev: float = None      # previous PS depth (for Vz differentiation)
        self.ps_time_prev: float = None   # timestamp of previous PS reading

        # --- Publishers ---
        self.state_pub = self.create_publisher(AuvState, f'/{self.imu_name}/state', 10)

        # --- Subscribers ---
        # Single combined ImuAccel topic (linear + angular acceleration)
        self.accel_sub = self.create_subscription(
            ImuAccel, f'/{self.imu_name}/accel', self.accel_callback, 10)
        # Pressure sensor for Z fusion
        self.ps_sub = self.create_subscription(
            PsData, '/pressure_sensor/data', self.ps_callback, 10)

        self.get_logger().info(f'{self.imu_name.upper()} Localization node initialized (3D + PS-Z fusion)')
        self.get_logger().info(f'  Bias (ax,ay,az,alpha): {self.bias}')
        self.get_logger().info(f'  R diag: {np.diag(self.R)}')
        self.get_logger().info(f'  PS weight for Z: {self.ps_weight_z:.2f}')

    @abstractmethod
    def _declare_imu_parameters(self):
        pass

    @abstractmethod
    def _load_noise_model(self) -> np.ndarray:
        """Return 4x4 measurement noise covariance for [ax, ay, az, alpha]"""
        pass

    @abstractmethod
    def _load_bias(self) -> np.ndarray:
        """Return bias vector [ax_bias, ay_bias, az_bias, alpha_bias]"""
        pass

    def ps_callback(self, msg: PsData):
        """Cache latest pressure sensor depth; compute Vz by differentiation."""
        now = self.get_clock().now().nanoseconds * 1e-9
        new_z = msg.depth

        if self.ps_z is not None and self.ps_time_prev is not None:
            dt_ps = now - self.ps_time_prev
            if 0 < dt_ps < 0.5:
                # Numerically differentiate PS depth → Vz estimate
                ps_vz = (new_z - self.ps_z) / dt_ps
                # Blend into state Vz only when PS data is fresh
                w = self.ps_weight_z
                self.state[6] = (1.0 - w) * self.state[6] + w * ps_vz

        self.ps_z_prev = self.ps_z
        self.ps_z = new_z
        self.ps_time_prev = now

    def accel_callback(self, msg: ImuAccel):
        """
        Process clean combined acceleration from simulator.
        linear  = [ax, ay, az]
        angular = [alpha, ?, ?]  — only x component used (yaw angular accel)

        Adds bias + measurement noise, then double-integrates 3D state.
        After integration, fuses PS depth into Z (PS as primary).
        """
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last_time is None:
            self.last_time = current_time
            return
        dt = current_time - self.last_time
        if dt <= 0 or dt > 0.1:
            self.last_time = current_time
            return

        # Clean accelerations from simulator
        a_clean = np.array([
            msg.linear.x,
            msg.linear.y,
            msg.linear.z,
            msg.angular.x,   # yaw angular accel
        ])

        # Add bias + measurement noise
        noise = np.random.multivariate_normal(np.zeros(4), self.R)
        a_corrupted = a_clean + self.bias + noise

        ax, ay, az, alpha = a_corrupted

        x, y, z, yaw, vx, vy, vz, wyaw = self.state

        # Integrate velocities
        vx += ax * dt
        vy += ay * dt
        vz += az * dt
        wyaw += alpha * dt

        # Integrate positions
        x += vx * dt
        y += vy * dt
        z += vz * dt
        yaw += wyaw * dt
        yaw = normalize_angle(yaw)

        # Z floor: estimate also clamps at surface
        if z < 0.0:
            z = 0.0
            if vz < 0.0:
                vz = 0.0

        self.state = np.array([x, y, z, yaw, vx, vy, vz, wyaw])

        # --- PS Z fusion (PS is primary/default for depth) ---
        if self.ps_z is not None:
            w = self.ps_weight_z
            fused_z = (1.0 - w) * self.state[2] + w * self.ps_z
            # Clamp fused Z at surface
            if fused_z < 0.0:
                fused_z = 0.0
            self.state[2] = fused_z
            # Note: Vz fusion is handled in ps_callback via differentiation

        self.last_time = current_time
        self.publish_state(msg.header.stamp)

    def publish_state(self, timestamp):
        """Publish 3D estimated state as AuvState"""
        x, y, z, yaw, vx, vy, vz, wyaw = self.state
        msg = AuvState()
        msg.header.stamp = timestamp
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
