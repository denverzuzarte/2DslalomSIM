#!/usr/bin/env python3
"""
Controller Node — Dual-mode controller (direct force / PID setpoint).

Topics:
  IN:  /{localization_source}/state  (AuvState)  — current estimated state
  IN:  /controller/force             (ImuAccel)  — direct 6D force command passthrough
  IN:  /controller/setpoint          (AuvState)  — PID target (full state setpoint)
  OUT: /controller/force             (ImuAccel)  — computed force [fx,fy,fz] in linear field

NOTE: /control/command and the old /control/force topics are removed.
The controller now publishes directly on /controller/force which the simulator
subscribes to as its force input.
"""
import rclpy
from rclpy.node import Node
import numpy as np

from auv_msgs.msg import AuvState, ImuAccel
from slalom_simulator.utils import normalize_angle


class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')

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

        self.localization_source = self.get_parameter('localization_source').value
        self.pid_kp = self.get_parameter('pid_kp').value
        self.pid_ki = self.get_parameter('pid_ki').value
        self.pid_kd = self.get_parameter('pid_kd').value
        self.max_control_force = self.get_parameter('max_control_force').value

        self.current_state: AuvState = None
        self.current_setpoint: AuvState = None   # full AuvState setpoint
        self.direct_force: np.ndarray = None     # [fx, fy, fz] from external override

        self.integral_error = np.zeros(2)
        self.last_error = np.zeros(2)

        # Output: ImuAccel on /controller/force (simulator listens here)
        self.force_pub = self.create_publisher(ImuAccel, '/controller/force', 10)

        state_topic = f'/{self.localization_source}/state'
        self.state_sub = self.create_subscription(
            AuvState, state_topic, self.state_callback, 10)

        # Direct force override: ImuAccel (linear.xyz = [fx, fy, fz])
        self.force_in_sub = self.create_subscription(
            ImuAccel, '/controller/force_in', self.force_in_callback, 10)

        # Setpoint: full AuvState (uses position.x, position.y for PID target)
        self.setpoint_sub = self.create_subscription(
            AuvState, '/controller/setpoint', self.setpoint_callback, 10)

        self.create_timer(0.05, self.control_loop)  # 20 Hz

        self.get_logger().info(
            f'Controller node initialized — source: {self.localization_source}')

    def state_callback(self, msg: AuvState):
        self.current_state = msg

    def force_in_callback(self, msg: ImuAccel):
        """Direct 6D force override — bypasses PID."""
        self.direct_force = np.array([msg.linear.x, msg.linear.y, msg.linear.z])
        self.current_setpoint = None

    def setpoint_callback(self, msg: AuvState):
        """Set new PID target from AuvState (uses position x,y)."""
        self.current_setpoint = msg
        self.direct_force = None
        self.integral_error = np.zeros(2)
        self.last_error = np.zeros(2)

    def control_loop(self):
        if self.current_state is None:
            return

        if self.direct_force is not None:
            force_xyz = self.direct_force.copy()
        elif self.current_setpoint is not None:
            force_xy = self._pid()
            mag = np.linalg.norm(force_xy)
            if mag > self.max_control_force:
                force_xy = force_xy / mag * self.max_control_force
            force_xyz = np.array([force_xy[0], force_xy[1], 0.0])
        else:
            force_xyz = np.zeros(3)

        # Overall clamp
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

    def _pid(self) -> np.ndarray:
        pos = np.array([
            self.current_state.position.x,
            self.current_state.position.y,
        ])
        target = np.array([
            self.current_setpoint.position.x,
            self.current_setpoint.position.y,
        ])
        error = target - pos
        self.integral_error += error
        d_term = error - self.last_error
        self.last_error = error.copy()
        return (self.pid_kp * error
                + self.pid_ki * self.integral_error
                + self.pid_kd * d_term)


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
