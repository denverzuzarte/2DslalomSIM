#!/usr/bin/env python3
"""
Main Pygame-based simulator node for 2D slalom navigation.
Simulates vehicle physics, IMU sensors, and visualization.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, Image
from geometry_msgs.msg import PoseStamped, Vector3, Quaternion
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header

import pygame
import numpy as np
import threading
import time
from cv_bridge import CvBridge
import cv2

from slalom_simulator.utils import (
    generate_gates, normalize_angle,
    COLOR_BACKGROUND, COLOR_VEHICLE, COLOR_GHOST,
    COLOR_RED_POLE, COLOR_WHITE_POLE, COLOR_FORCE_ARROW
)


class SimulatorNode(Node):
    def __init__(self):
        super().__init__('simulator_node')

        # Declare and get parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('pool_width', 600),
                ('pool_height', 800),
                ('vehicle_size', 40),
                ('vehicle_start_x', 400.0),
                ('vehicle_start_y', 500.0),
                ('imu1_bias', [0.01, 0.01, 0.001]),
                ('imu2_bias', [0.05, 0.05, 0.005]),
                ('imu1_covariance', [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.001]),
                ('imu2_covariance', [0.05, 0.0, 0.0, 0.0, 0.05, 0.0, 0.0, 0.0, 0.005]),
                ('num_gates', 3),
                ('gate_width_mean', 70.0),
                ('gate_width_std', 5.0),
                ('pole_separation_mean', 100.0),
                ('pole_separation_std', 5.0),
            ]
        )

        # Get parameters
        self.pool_width = self.get_parameter('pool_width').value
        self.pool_height = self.get_parameter('pool_height').value
        self.vehicle_size = self.get_parameter('vehicle_size').value

        # Vehicle state: [x, y, yaw, vx, vy, omega]
        self.state = np.array([
            self.get_parameter('vehicle_start_x').value,
            self.get_parameter('vehicle_start_y').value,
            0.0,  # yaw
            0.0,  # vx
            0.0,  # vy
            0.0   # omega
        ])

        # IMU biases
        self.imu1_bias = np.array(self.get_parameter('imu1_bias').value)
        self.imu2_bias = np.array(self.get_parameter('imu2_bias').value)

        # IMU covariances
        self.imu1_cov = np.array(self.get_parameter('imu1_covariance').value).reshape(3, 3)
        self.imu2_cov = np.array(self.get_parameter('imu2_covariance').value).reshape(3, 3)

        # Vehicle dynamics parameters
        self.mass = 1.0
        self.damping = 0.1
        self.dt = 1.0 / 60.0  # 60 Hz simulation

        # Control force
        self.control_force = np.array([0.0, 0.0])

        # Generate gates
        self.gates = generate_gates(
            num_gates=self.get_parameter('num_gates').value,
            pool_width=self.pool_width,
            pool_height=self.pool_height,
            gate_width_mean=self.get_parameter('gate_width_mean').value,
            gate_width_std=self.get_parameter('gate_width_std').value,
            pole_separation_mean=self.get_parameter('pole_separation_mean').value,
            pole_separation_std=self.get_parameter('pole_separation_std').value
        )

        # Ghost vehicle poses
        self.imu1_pose = None
        self.imu2_pose = None
        self.kalman_pose = None

        # Initialize Pygame
        pygame.init()
        self.screen = pygame.display.set_mode((self.pool_width, self.pool_height))
        pygame.display.set_caption("2D Slalom Simulator")
        self.clock = pygame.time.Clock()
        self.font = pygame.font.Font(None, 20)

        # CV Bridge for image conversion
        self.bridge = CvBridge()

        # Publishers
        self.imu1_pub = self.create_publisher(Imu, '/imu1/data', 10)
        self.imu2_pub = self.create_publisher(Imu, '/imu2/data', 10)
        self.gt_pub = self.create_publisher(PoseStamped, '/ground_truth', 10)
        self.camera_pub = self.create_publisher(Image, '/camera/image', 10)
        self.force_pub = self.create_publisher(Vector3, '/control/force', 10)
        self.gates_pub = self.create_publisher(MarkerArray, '/slalom/gates', 10)

        # Subscribers
        self.control_sub = self.create_subscription(
            Vector3, '/control/command', self.control_callback, 10)
        self.imu1_pose_sub = self.create_subscription(
            PoseStamped, '/imu1/pose', self.imu1_pose_callback, 10)
        self.imu2_pose_sub = self.create_subscription(
            PoseStamped, '/imu2/pose', self.imu2_pose_callback, 10)
        self.kalman_pose_sub = self.create_subscription(
            PoseStamped, '/kalman/pose', self.kalman_pose_callback, 10)

        # Timers
        self.create_timer(1.0 / 60.0, self.simulation_step)  # 60 Hz
        self.create_timer(1.0 / 30.0, self.publish_camera)   # 30 Hz
        self.create_timer(1.0, self.publish_gates)            # 1 Hz

        # Start IMU publishing thread
        self.imu_thread = threading.Thread(target=self.imu_publishing_loop, daemon=True)
        self.imu_running = True
        self.imu_thread.start()

        # Last acceleration for IMU
        self.last_accel = np.array([0.0, 0.0])
        self.last_angular_accel = 0.0

        self.get_logger().info('Simulator node initialized')
        self.publish_gates()  # Publish gates immediately

    def control_callback(self, msg):
        """Receive control force command"""
        self.control_force = np.array([msg.x, msg.y])

    def imu1_pose_callback(self, msg):
        """Receive IMU1 pose estimate for ghost rendering"""
        self.imu1_pose = msg

    def imu2_pose_callback(self, msg):
        """Receive IMU2 pose estimate for ghost rendering"""
        self.imu2_pose = msg

    def kalman_pose_callback(self, msg):
        """Receive Kalman filter pose estimate for ghost rendering"""
        self.kalman_pose = msg

    def simulation_step(self):
        """Main simulation step at 60 Hz"""
        # Handle Pygame events
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.imu_running = False
                rclpy.shutdown()
                return

        # Update vehicle dynamics
        self.update_dynamics()

        # Render
        self.render()

        # Publish ground truth
        self.publish_ground_truth()

        # Publish control force
        self.publish_control_force()

        # Maintain frame rate
        self.clock.tick(60)

    def update_dynamics(self):
        """Update vehicle physics using semi-implicit Euler integration"""
        x, y, yaw, vx, vy, omega = self.state

        # Apply control force and damping
        ax = (self.control_force[0] - self.damping * vx) / self.mass
        ay = (self.control_force[1] - self.damping * vy) / self.mass
        alpha = -self.damping * omega  # Angular damping

        # Store for IMU
        self.last_accel = np.array([ax, ay])
        self.last_angular_accel = alpha

        # Update velocities
        vx += ax * self.dt
        vy += ay * self.dt
        omega += alpha * self.dt

        # Update positions
        x += vx * self.dt
        y += vy * self.dt
        yaw += omega * self.dt
        yaw = normalize_angle(yaw)

        # Boundary constraints (bounce off walls)
        if x < self.vehicle_size / 2:
            x = self.vehicle_size / 2
            vx = -0.5 * vx
        elif x > self.pool_width - self.vehicle_size / 2:
            x = self.pool_width - self.vehicle_size / 2
            vx = -0.5 * vx

        if y < self.vehicle_size / 2:
            y = self.vehicle_size / 2
            vy = -0.5 * vy
        elif y > self.pool_height - self.vehicle_size / 2:
            y = self.pool_height - self.vehicle_size / 2
            vy = -0.5 * vy

        self.state = np.array([x, y, yaw, vx, vy, omega])

    def render(self):
        """Render the simulation using Pygame"""
        # Clear screen
        self.screen.fill(COLOR_BACKGROUND)

        # Draw gates
        for gate in self.gates:
            red_pos = gate['red_pole'].astype(int)
            white_pos = gate['white_pole'].astype(int)
            pygame.draw.circle(self.screen, COLOR_RED_POLE, red_pos, 4)
            pygame.draw.circle(self.screen, COLOR_WHITE_POLE, white_pos, 4)

        # Draw ghost vehicles
        self.draw_ghost(self.imu1_pose, "IMU1")
        self.draw_ghost(self.imu2_pose, "IMU2")
        self.draw_ghost(self.kalman_pose, "KF")

        # Draw main vehicle
        x, y, yaw, _, _, _ = self.state
        half_size = self.vehicle_size / 2

        # Calculate vehicle corners
        corners = np.array([
            [-half_size, -half_size],
            [half_size, -half_size],
            [half_size, half_size],
            [-half_size, half_size]
        ])

        # Rotate corners
        cos_yaw = np.cos(yaw)
        sin_yaw = np.sin(yaw)
        rotation = np.array([[cos_yaw, -sin_yaw], [sin_yaw, cos_yaw]])
        rotated_corners = (rotation @ corners.T).T + np.array([x, y])

        pygame.draw.polygon(self.screen, COLOR_VEHICLE, rotated_corners.astype(int))

        # Draw control force arrow
        if np.linalg.norm(self.control_force) > 0.1:
            self.draw_arrow(x, y, self.control_force)

        # Update display
        pygame.display.flip()

    def draw_ghost(self, pose_msg, label):
        """Draw a semi-transparent ghost vehicle"""
        if pose_msg is None:
            return

        x = pose_msg.pose.position.x
        y = pose_msg.pose.position.y

        # Extract yaw from quaternion
        q = pose_msg.pose.orientation
        yaw = np.arctan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))

        half_size = self.vehicle_size / 2

        # Calculate corners
        corners = np.array([
            [-half_size, -half_size],
            [half_size, -half_size],
            [half_size, half_size],
            [-half_size, half_size]
        ])

        # Rotate
        cos_yaw = np.cos(yaw)
        sin_yaw = np.sin(yaw)
        rotation = np.array([[cos_yaw, -sin_yaw], [sin_yaw, cos_yaw]])
        rotated_corners = (rotation @ corners.T).T + np.array([x, y])

        # Create transparent surface
        ghost_surface = pygame.Surface((self.pool_width, self.pool_height), pygame.SRCALPHA)
        pygame.draw.polygon(ghost_surface, (*COLOR_GHOST, 128), rotated_corners.astype(int))
        self.screen.blit(ghost_surface, (0, 0))

        # Draw label
        text = self.font.render(label, True, (0, 0, 0))
        self.screen.blit(text, (int(x) - 15, int(y) - half_size - 20))

    def draw_arrow(self, x, y, force):
        """Draw force arrow from vehicle center"""
        scale = 5.0  # Visual scale for arrow
        arrow_end = np.array([x, y]) + force * scale

        # Draw line
        pygame.draw.line(self.screen, COLOR_FORCE_ARROW,
                        (int(x), int(y)),
                        (int(arrow_end[0]), int(arrow_end[1])), 3)

        # Draw arrowhead
        angle = np.arctan2(force[1], force[0])
        arrow_size = 10
        left_angle = angle + 2.5
        right_angle = angle - 2.5

        left_point = arrow_end + arrow_size * np.array([np.cos(left_angle), np.sin(left_angle)])
        right_point = arrow_end + arrow_size * np.array([np.cos(right_angle), np.sin(right_angle)])

        pygame.draw.polygon(self.screen, COLOR_FORCE_ARROW,
                          [arrow_end.astype(int), left_point.astype(int), right_point.astype(int)])

    def imu_publishing_loop(self):
        """Publish IMU data at 100 Hz in separate thread"""
        rate = 100.0  # Hz
        period = 1.0 / rate

        while self.imu_running and rclpy.ok():
            start_time = time.time()

            # Compute true acceleration in body frame
            ax, ay = self.last_accel
            alpha = self.last_angular_accel

            # Generate IMU measurements with bias and noise
            imu1_accel = np.array([ax, ay, alpha]) + self.imu1_bias + np.random.multivariate_normal([0, 0, 0], self.imu1_cov)
            imu2_accel = np.array([ax, ay, alpha]) + self.imu2_bias + np.random.multivariate_normal([0, 0, 0], self.imu2_cov)

            # Publish IMU1
            imu1_msg = Imu()
            imu1_msg.header = Header()
            imu1_msg.header.stamp = self.get_clock().now().to_msg()
            imu1_msg.header.frame_id = 'imu1_frame'
            imu1_msg.linear_acceleration.x = float(imu1_accel[0])
            imu1_msg.linear_acceleration.y = float(imu1_accel[1])
            imu1_msg.linear_acceleration.z = 0.0
            imu1_msg.angular_velocity.x = 0.0
            imu1_msg.angular_velocity.y = 0.0
            imu1_msg.angular_velocity.z = float(self.state[5])  # omega
            self.imu1_pub.publish(imu1_msg)

            # Publish IMU2
            imu2_msg = Imu()
            imu2_msg.header = Header()
            imu2_msg.header.stamp = self.get_clock().now().to_msg()
            imu2_msg.header.frame_id = 'imu2_frame'
            imu2_msg.linear_acceleration.x = float(imu2_accel[0])
            imu2_msg.linear_acceleration.y = float(imu2_accel[1])
            imu2_msg.linear_acceleration.z = 0.0
            imu2_msg.angular_velocity.x = 0.0
            imu2_msg.angular_velocity.y = 0.0
            imu2_msg.angular_velocity.z = float(self.state[5])  # omega
            self.imu2_pub.publish(imu2_msg)

            # Sleep to maintain rate
            elapsed = time.time() - start_time
            sleep_time = period - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    def publish_ground_truth(self):
        """Publish ground truth pose"""
        msg = PoseStamped()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'

        x, y, yaw, _, _, _ = self.state
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = 0.0

        # Convert yaw to quaternion
        msg.pose.orientation.w = float(np.cos(yaw / 2))
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = float(np.sin(yaw / 2))

        self.gt_pub.publish(msg)

    def publish_control_force(self):
        """Publish current control force"""
        msg = Vector3()
        msg.x = float(self.control_force[0])
        msg.y = float(self.control_force[1])
        msg.z = 0.0
        self.force_pub.publish(msg)

    def publish_camera(self):
        """Publish camera image from Pygame surface"""
        # Convert Pygame surface to numpy array
        frame = pygame.surfarray.array3d(self.screen)
        # Transpose to correct orientation (Pygame uses (width, height, channels))
        frame = np.transpose(frame, (1, 0, 2))
        # Convert RGB to BGR for OpenCV
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        # Convert to ROS Image message
        try:
            img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            img_msg.header = Header()
            img_msg.header.stamp = self.get_clock().now().to_msg()
            img_msg.header.frame_id = 'camera_frame'
            self.camera_pub.publish(img_msg)
        except Exception as e:
            self.get_logger().error(f'Failed to publish camera image: {e}')

    def publish_gates(self):
        """Publish gate markers"""
        marker_array = MarkerArray()

        for i, gate in enumerate(self.gates):
            # Red pole marker
            red_marker = Marker()
            red_marker.header = Header()
            red_marker.header.stamp = self.get_clock().now().to_msg()
            red_marker.header.frame_id = 'world'
            red_marker.ns = 'gates'
            red_marker.id = i * 2
            red_marker.type = Marker.SPHERE
            red_marker.action = Marker.ADD
            red_marker.pose.position.x = float(gate['red_pole'][0])
            red_marker.pose.position.y = float(gate['red_pole'][1])
            red_marker.pose.position.z = 0.0
            red_marker.pose.orientation.w = 1.0
            red_marker.scale.x = 8.0
            red_marker.scale.y = 8.0
            red_marker.scale.z = 8.0
            red_marker.color.r = 1.0
            red_marker.color.g = 0.0
            red_marker.color.b = 0.0
            red_marker.color.a = 1.0
            marker_array.markers.append(red_marker)

            # White pole marker
            white_marker = Marker()
            white_marker.header = red_marker.header
            white_marker.ns = 'gates'
            white_marker.id = i * 2 + 1
            white_marker.type = Marker.SPHERE
            white_marker.action = Marker.ADD
            white_marker.pose.position.x = float(gate['white_pole'][0])
            white_marker.pose.position.y = float(gate['white_pole'][1])
            white_marker.pose.position.z = 0.0
            white_marker.pose.orientation.w = 1.0
            white_marker.scale.x = 8.0
            white_marker.scale.y = 8.0
            white_marker.scale.z = 8.0
            white_marker.color.r = 1.0
            white_marker.color.g = 1.0
            white_marker.color.b = 1.0
            white_marker.color.a = 1.0
            marker_array.markers.append(white_marker)

        self.gates_pub.publish(marker_array)

    def destroy_node(self):
        """Clean up before shutdown"""
        self.imu_running = False
        pygame.quit()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SimulatorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
