#!/usr/bin/env python3
"""
Controller Node - Dual-mode controller supporting direct force and PID setpoint control.

Modes:
1. Direct Force Mode: Accepts force commands from /controller/force
2. PID Setpoint Mode: Accepts position setpoints from /controller/setpoint and computes force via PID
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from std_msgs.msg import Header
import numpy as np

from slalom_simulator.msg import State, Setpoint
from slalom_simulator.utils import normalize_angle


class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')

        # Declare and get parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('localization_source', 'imu1'),
                ('pid_kp', 0.5),
                ('pid_ki', 0.01),
                ('pid_kd', 0.1),
                ('max_control_force', 10.0),
            ]
        )

        # Get parameters
        self.localization_source = self.get_parameter('localization_source').value
        self.pid_kp = self.get_parameter('pid_kp').value
        self.pid_ki = self.get_parameter('pid_ki').value
        self.pid_kd = self.get_parameter('pid_kd').value
        self.max_control_force = self.get_parameter('max_control_force').value

        # State
        self.current_state = None
        self.current_setpoint = None
        self.direct_force = None

        # PID state
        self.integral_error = np.array([0.0, 0.0])
        self.last_error = np.array([0.0, 0.0])
        self.last_yaw_error = 0.0
        self.integral_yaw_error = 0.0

        # Publisher
        self.control_pub = self.create_publisher(Vector3, '/control/command', 10)

        # Subscribers
        state_topic = f'/{self.localization_source}/state'
        self.state_sub = self.create_subscription(
            State, state_topic, self.state_callback, 10)

        self.force_sub = self.create_subscription(
            Vector3, '/controller/force', self.force_callback, 10)

        self.setpoint_sub = self.create_subscription(
            Setpoint, '/controller/setpoint', self.setpoint_callback, 10)

        # Timer for control loop
        self.create_timer(0.05, self.control_loop)  # 20 Hz

        self.get_logger().info(f'Controller node initialized')
        self.get_logger().info(f'  Localization source: {self.localization_source}')
        self.get_logger().info(f'  Modes: Direct force (/controller/force) or PID setpoint (/controller/setpoint)')

    def state_callback(self, msg):
        """Receive state estimate from selected localization source"""
        self.current_state = msg

    def force_callback(self, msg):
        """Receive direct force command (overrides setpoint mode)"""
        self.direct_force = np.array([msg.x, msg.y])
        # Clear setpoint when direct force is commanded
        self.current_setpoint = None

    def setpoint_callback(self, msg):
        """Receive position setpoint (activates PID mode)"""
        self.current_setpoint = msg
        # Clear direct force when setpoint is commanded
        self.direct_force = None
        # Reset PID integrators
        self.integral_error = np.array([0.0, 0.0])
        self.integral_yaw_error = 0.0

    def control_loop(self):
        """Main control loop - runs at 20 Hz"""
        if self.current_state is None:
            return

        control_force = np.array([0.0, 0.0])

        # Mode selection: Direct force takes precedence
        if self.direct_force is not None:
            # Direct force mode
            control_force = self.direct_force.copy()
        elif self.current_setpoint is not None:
            # PID setpoint mode
            control_force = self.compute_pid_force()
        # else: No command, zero force

        # Limit control force
        force_magnitude = np.linalg.norm(control_force)
        if force_magnitude > self.max_control_force:
            control_force = control_force / force_magnitude * self.max_control_force

        # Publish control command
        self.publish_control(control_force)

    def compute_pid_force(self):
        """
        Compute control force using PID to reach setpoint.

        Returns:
            np.ndarray: [fx, fy] control force
        """
        # Current state
        x = self.current_state.x
        y = self.current_state.y
        yaw = self.current_state.yaw
        current_pos = np.array([x, y])

        # Target setpoint
        target_pos = np.array([self.current_setpoint.x, self.current_setpoint.y])
        target_yaw = self.current_setpoint.yaw

        # Position error
        error = target_pos - current_pos

        # Proportional
        p_term = self.pid_kp * error

        # Integral
        self.integral_error += error
        i_term = self.pid_ki * self.integral_error

        # Derivative
        d_term = self.pid_kd * (error - self.last_error)
        self.last_error = error.copy()

        # Combined PID force for position
        pid_force = p_term + i_term + d_term

        # TODO: Optionally add yaw control (would need to convert to torque)
        # For now, we only control x, y position

        return pid_force

    def publish_control(self, force):
        """Publish control command"""
        msg = Vector3()
        msg.x = float(force[0])
        msg.y = float(force[1])
        msg.z = 0.0
        self.control_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
