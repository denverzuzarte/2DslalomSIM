#!/usr/bin/env python3
"""
Base IMU Localization Node - Abstract base class for IMU-based dead reckoning.

This base class provides common functionality for IMU localization nodes.
Each IMU node subscribes to CLEAN acceleration data from the simulator and adds
its own measurement noise (R matrix) and bias to simulate sensor imperfection.
Then it performs double integration to estimate the vehicle state.

This causes drift over time due to:
1. Persistent bias (linear velocity error growth)
2. Measurement noise integration (random walk in position)
3. Process noise from simulator (affects true state)
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Header
import numpy as np
from abc import ABC, abstractmethod

from slalom_interfaces.msg import State
from slalom_simulator.utils import load_covariance_matrix_3x3, normalize_angle


class BaseIMULocalizationNode(Node, ABC):
    """Abstract base class for IMU localization nodes"""

    def __init__(self, node_name: str, imu_name: str):
        """
        Initialize the base IMU localization node.

        Args:
            node_name: ROS2 node name
            imu_name: IMU identifier (e.g., 'imu1', 'imu2')
        """
        super().__init__(node_name)

        self.imu_name = imu_name

        # Declare and get common parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('vehicle_start_x', 400.0),
                ('vehicle_start_y', 500.0),
            ]
        )

        # Declare IMU-specific parameters
        self._declare_imu_parameters()

        # Initialize state [x, y, yaw, vx, vy, omega]
        start_x = self.get_parameter('vehicle_start_x').value
        start_y = self.get_parameter('vehicle_start_y').value
        self.state = np.array([start_x, start_y, 0.0, 0.0, 0.0, 0.0])

        # Load IMU noise model and bias
        self.R = self._load_noise_model()
        self.bias = self._load_bias()

        # Last timestamp for integration
        self.last_time = None

        # Publisher - State message
        self.state_pub = self.create_publisher(State, f'/{self.imu_name}/state', 10)

        # Subscriber - Clean acceleration from simulator
        self.accel_sub = self.create_subscription(
            Vector3Stamped, f'/{self.imu_name}/accel', self.accel_callback, 10)

        self.get_logger().info(f'{self.imu_name.upper()} Localization node initialized')
        self.get_logger().info(f'  Bias: {self.bias}')
        self.get_logger().info(f'  R matrix diag: [{self.R[0,0]:.4f}, {self.R[1,1]:.4f}, {self.R[2,2]:.4f}]')

    @abstractmethod
    def _declare_imu_parameters(self):
        """Declare IMU-specific parameters (R matrix, bias)"""
        pass

    @abstractmethod
    def _load_noise_model(self) -> np.ndarray:
        """Load and return the R (measurement noise) matrix"""
        pass

    @abstractmethod
    def _load_bias(self) -> np.ndarray:
        """Load and return the bias vector [ax_bias, ay_bias, alpha_bias]"""
        pass

    def accel_callback(self, msg):
        """
        Process clean acceleration data from simulator.

        CRITICAL: The simulator publishes CLEAN acceleration (true dynamics).
        This node adds measurement noise and bias to simulate sensor imperfection.
        """
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        if self.last_time is None:
            self.last_time = current_time
            return

        dt = current_time - self.last_time
        if dt <= 0 or dt > 0.1:  # Sanity check
            self.last_time = current_time
            return

        # Get clean acceleration from simulator
        a_clean = np.array([msg.vector.x, msg.vector.y, msg.vector.z])

        # Add persistent bias (constant for this IMU)
        a_biased = a_clean + self.bias

        # Add measurement noise (sample from N(0, R) at each timestep)
        noise = np.random.multivariate_normal([0, 0, 0], self.R)
        a_corrupted = a_biased + noise

        # Extract components
        ax, ay, alpha = a_corrupted

        # Current state
        x, y, yaw, vx, vy, omega = self.state

        # Double integration: a → v → p
        # This ALWAYS causes drift due to bias + noise accumulation

        # Update velocities using corrupted accelerations
        vx += ax * dt
        vy += ay * dt
        omega += alpha * dt

        # Update position using velocities
        x += vx * dt
        y += vy * dt
        yaw += omega * dt
        yaw = normalize_angle(yaw)

        # Store updated state
        self.state = np.array([x, y, yaw, vx, vy, omega])
        self.last_time = current_time

        # Publish state estimate
        self.publish_state(msg.header.stamp)

    def publish_state(self, timestamp):
        """Publish estimated state"""
        msg = State()
        msg.header = Header()
        msg.header.stamp = timestamp
        msg.header.frame_id = 'world'

        x, y, yaw, vx, vy, omega = self.state
        msg.x = float(x)
        msg.y = float(y)
        msg.yaw = float(yaw)
        msg.vx = float(vx)
        msg.vy = float(vy)
        msg.wyaw = float(omega)

        self.state_pub.publish(msg)
