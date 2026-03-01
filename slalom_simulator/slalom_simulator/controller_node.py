#!/usr/bin/env python3
"""
Controller Node — Dual-mode controller (direct force / PID setpoint).

Error is computed as:
  e = setpoint (AuvState) - ground_truth (AuvState)
for all 8 DOF: x, y, z, yaw, vx, vy, vz, wyaw.

PID implemented with the simple-pid library (one PID instance per DOF).

Topics:
  IN:  /ground_truth              (AuvState)  — true vehicle state (error reference)
  IN:  /controller/force_in       (ImuAccel)  — direct 6D force override (bypasses PID)
  IN:  /controller/setpoint       (AuvState)  — PID target (full 8-DOF setpoint)
  OUT: /controller/force          (ImuAccel)  — computed force [fx,fy,fz] in linear field
"""
import rclpy
from rclpy.node import Node
import numpy as np
from simple_pid import PID

from auv_msgs.msg import AuvState, ImuAccel
from slalom_simulator.utils import normalize_angle


# DOF indices into the 8-element state vector
_X, _Y, _Z, _YAW, _VX, _VY, _VZ, _WYAW = range(8)


def _state_to_vec(msg: AuvState) -> np.ndarray:
    """Flatten AuvState into [x, y, z, yaw, vx, vy, vz, wyaw]."""
    return np.array([
        msg.position.x,
        msg.position.y,
        msg.position.z,
        msg.orientation.yaw,
        msg.velocity.x,
        msg.velocity.y,
        msg.velocity.z,
        msg.angular_velocity.z,
    ])


class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('pid_kp', 0.5),
                ('pid_ki', 0.01),
                ('pid_kd', 0.1),
                ('max_control_force', 10.0),
            ]
        )

        kp = self.get_parameter('pid_kp').value
        ki = self.get_parameter('pid_ki').value
        kd = self.get_parameter('pid_kd').value
        self.max_control_force = self.get_parameter('max_control_force').value

        dt = 1.0 / 100.0   # 100 Hz

        # One PID per DOF: x, y, z, yaw, vx, vy, vz, wyaw
        # Output limits symmetric around 0; scaled to max_control_force
        lim = self.max_control_force
        self.pids = [
            PID(kp, ki, kd, setpoint=0.0, sample_time=dt,
                output_limits=(-lim, lim))
            for _ in range(8)
        ]

        self.ground_truth: AuvState = None
        self.setpoint: AuvState = None
        self.direct_force: np.ndarray = None

        # Output
        self.force_pub = self.create_publisher(ImuAccel, '/controller/force', 10)

        # Subscriptions
        self.gt_sub = self.create_subscription(
            AuvState, '/ground_truth', self.gt_callback, 10)
        self.force_in_sub = self.create_subscription(
            ImuAccel, '/controller/force_in', self.force_in_callback, 10)
        self.setpoint_sub = self.create_subscription(
            AuvState, '/controller/setpoint', self.setpoint_callback, 10)

        self.create_timer(dt, self.control_loop)   # 100 Hz

        self.get_logger().info('Controller node initialized — 100 Hz, error = setpoint - ground_truth')

    def gt_callback(self, msg: AuvState):
        self.ground_truth = msg

    def force_in_callback(self, msg: ImuAccel):
        """Direct 6D force override — bypasses PID."""
        self.direct_force = np.array([msg.linear.x, msg.linear.y, msg.linear.z])
        self.setpoint = None

    def setpoint_callback(self, msg: AuvState):
        """New PID target; reset all integrators."""
        self.setpoint = msg
        self.direct_force = None
        sp = _state_to_vec(msg)
        for i, pid in enumerate(self.pids):
            pid.setpoint = sp[i]
            pid.reset()   # clears integral + last error

    def control_loop(self):
        if self.ground_truth is None:
            return

        if self.direct_force is not None:
            force_xyz = self.direct_force.copy()

        elif self.setpoint is not None:
            gt = _state_to_vec(self.ground_truth)

            # Feed current measurement into each PID
            # simple-pid: output = kp*e + ki*∫e + kd*de/dt  where e = setpoint - measurement
            outputs = np.array([
                self.pids[i](gt[i]) for i in range(8)
            ])

            # Map PID outputs to [fx, fy, fz] — positions/velocities drive forces
            # XY: position error + velocity error
            fx = outputs[_X] + outputs[_VX]
            fy = outputs[_Y] + outputs[_VY]
            fz = outputs[_Z] + outputs[_VZ]

            force_xyz = np.array([fx, fy, fz])

        else:
            force_xyz = np.zeros(3)

        # Clamp magnitude
        mag = np.linalg.norm(force_xyz)
        if mag > self.max_control_force:
            force_xyz = force_xyz / mag * self.max_control_force

        msg = ImuAccel()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        msg.linear.x = float(force_xyz[0])
        msg.linear.y = float(force_xyz[1])
        msg.linear.z = float(force_xyz[2])
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
        self.force_pub.publish(msg)


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

