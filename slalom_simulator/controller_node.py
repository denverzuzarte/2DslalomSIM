#!/usr/bin/env python3
"""
Controller Node - PID controller with Artificial Potential Field (APF).
Generates control forces to navigate the vehicle through slalom gates.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Vector3
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import Header
import numpy as np


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
                ('apf_attractive_gain', 2.0),
                ('apf_repulsive_gain', 50.0),
                ('apf_repulsive_distance', 50.0),
                ('max_control_force', 10.0),
            ]
        )

        # Get parameters
        self.localization_source = self.get_parameter('localization_source').value
        self.pid_kp = self.get_parameter('pid_kp').value
        self.pid_ki = self.get_parameter('pid_ki').value
        self.pid_kd = self.get_parameter('pid_kd').value
        self.apf_attractive_gain = self.get_parameter('apf_attractive_gain').value
        self.apf_repulsive_gain = self.get_parameter('apf_repulsive_gain').value
        self.apf_repulsive_distance = self.get_parameter('apf_repulsive_distance').value
        self.max_control_force = self.get_parameter('max_control_force').value

        # State
        self.current_pose = None
        self.gates = []
        self.current_gate_index = 0

        # PID state
        self.integral_error = np.array([0.0, 0.0])
        self.last_error = np.array([0.0, 0.0])

        # Publisher
        self.control_pub = self.create_publisher(Vector3, '/control/command', 10)

        # Subscribers
        pose_topic = f'/{self.localization_source}/pose'
        self.pose_sub = self.create_subscription(
            PoseStamped, pose_topic, self.pose_callback, 10)
        self.gates_sub = self.create_subscription(
            MarkerArray, '/slalom/gates', self.gates_callback, 10)

        # Timer for control loop
        self.create_timer(0.05, self.control_loop)  # 20 Hz

        self.get_logger().info(f'Controller node initialized with localization source: {self.localization_source}')

    def pose_callback(self, msg):
        """Receive pose estimate from selected localization source"""
        self.current_pose = msg

    def gates_callback(self, msg):
        """Receive gate positions"""
        if len(msg.markers) == 0:
            return

        # Parse gate markers (every 2 markers form a gate: red and white poles)
        self.gates = []
        for i in range(0, len(msg.markers), 2):
            if i + 1 < len(msg.markers):
                red_pole = np.array([
                    msg.markers[i].pose.position.x,
                    msg.markers[i].pose.position.y
                ])
                white_pole = np.array([
                    msg.markers[i + 1].pose.position.x,
                    msg.markers[i + 1].pose.position.y
                ])
                # Gate center is midpoint between poles
                center = (red_pole + white_pole) / 2.0
                self.gates.append({
                    'center': center,
                    'red_pole': red_pole,
                    'white_pole': white_pole
                })

        self.get_logger().info(f'Received {len(self.gates)} gates', once=True)

    def control_loop(self):
        """Main control loop - runs at 20 Hz"""
        if self.current_pose is None or len(self.gates) == 0:
            return

        # Extract current position
        x = self.current_pose.pose.position.x
        y = self.current_pose.pose.position.y
        current_pos = np.array([x, y])

        # Get target gate
        if self.current_gate_index >= len(self.gates):
            # Completed all gates - hold position
            control_force = np.array([0.0, 0.0])
        else:
            target_gate = self.gates[self.current_gate_index]
            target_pos = target_gate['center']

            # Check if passed current gate
            distance_to_gate = np.linalg.norm(current_pos - target_pos)
            if distance_to_gate < 30.0:  # Threshold for gate passing
                self.current_gate_index += 1
                self.get_logger().info(f'Passed gate {self.current_gate_index}')
                self.integral_error = np.array([0.0, 0.0])  # Reset integral
                return

            # Compute APF-based control force
            control_force = self.compute_apf_force(current_pos, target_gate)

            # Apply PID control
            control_force = self.apply_pid(control_force, current_pos, target_pos)

        # Limit control force
        force_magnitude = np.linalg.norm(control_force)
        if force_magnitude > self.max_control_force:
            control_force = control_force / force_magnitude * self.max_control_force

        # Publish control command
        self.publish_control(control_force)

    def compute_apf_force(self, current_pos, target_gate):
        """
        Compute control force using Artificial Potential Field (APF).

        Attractive force: pulls vehicle toward gate center
        Repulsive force: pushes vehicle away from poles
        """
        # Attractive force toward gate center
        target_pos = target_gate['center']
        distance_to_target = np.linalg.norm(target_pos - current_pos)
        if distance_to_target > 0:
            attractive_force = self.apf_attractive_gain * (target_pos - current_pos)
        else:
            attractive_force = np.array([0.0, 0.0])

        # Repulsive forces from all poles
        repulsive_force = np.array([0.0, 0.0])

        for gate in self.gates:
            # Red pole repulsion
            red_pole = gate['red_pole']
            dist_red = np.linalg.norm(current_pos - red_pole)
            if dist_red < self.apf_repulsive_distance and dist_red > 0:
                repulsion_magnitude = self.apf_repulsive_gain * (1.0 / dist_red - 1.0 / self.apf_repulsive_distance) * (1.0 / dist_red**2)
                repulsive_force += repulsion_magnitude * (current_pos - red_pole) / dist_red

            # White pole repulsion
            white_pole = gate['white_pole']
            dist_white = np.linalg.norm(current_pos - white_pole)
            if dist_white < self.apf_repulsive_distance and dist_white > 0:
                repulsion_magnitude = self.apf_repulsive_gain * (1.0 / dist_white - 1.0 / self.apf_repulsive_distance) * (1.0 / dist_white**2)
                repulsive_force += repulsion_magnitude * (current_pos - white_pole) / dist_white

        # Total force
        total_force = attractive_force + repulsive_force
        return total_force

    def apply_pid(self, force, current_pos, target_pos):
        """Apply PID control to the force"""
        # Error
        error = target_pos - current_pos

        # Proportional
        p_term = self.pid_kp * error

        # Integral
        self.integral_error += error
        i_term = self.pid_ki * self.integral_error

        # Derivative
        d_term = self.pid_kd * (error - self.last_error)
        self.last_error = error.copy()

        # Combine
        pid_force = p_term + i_term + d_term
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
