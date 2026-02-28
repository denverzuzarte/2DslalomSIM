#!/usr/bin/env python3
"""
Main Pygame-based simulator node for 2D slalom navigation.
Simulates vehicle physics, IMU sensors, and visualization.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3Stamped, Vector3
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header
from sensor_msgs.msg import Image

import pygame
import numpy as np
import threading
import time
from cv_bridge import CvBridge
import cv2

from slalom_simulator.utils import (
    generate_gates, normalize_angle, build_Q_matrix,
    COLOR_BACKGROUND, COLOR_VEHICLE, COLOR_GHOST,
    COLOR_RED_POLE, COLOR_WHITE_POLE, COLOR_FORCE_ARROW
)

# Import custom messages (will be generated)
from slalom_interfaces.msg import State, CameraDetections, CameraDetection


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
                # Process noise parameters
                ('process_noise.sigma_ax_squared', 0.01),
                ('process_noise.sigma_ay_squared', 0.01),
                ('process_noise.sigma_alpha_squared', 0.001),
                # IMU parameters (kept for simulator to generate data)
                ('imu1_bias', [0.01, 0.01, 0.001]),
                ('imu2_bias', [0.05, 0.05, 0.005]),
                # Gates
                ('num_gates', 3),
                ('gate_width_mean', 70.0),
                ('gate_width_std', 5.0),
                ('pole_separation_mean', 100.0),
                ('pole_separation_std', 5.0),
                # Camera
                ('camera_fov_degrees', 60.0),
                ('camera_fps', 8.0),
                ('camera_pixel_variance', 4.0),
                ('camera_range_max', 200.0),
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

        # Process noise parameters
        self.sigma_ax2 = self.get_parameter('process_noise.sigma_ax_squared').value
        self.sigma_ay2 = self.get_parameter('process_noise.sigma_ay_squared').value
        self.sigma_alpha2 = self.get_parameter('process_noise.sigma_alpha_squared').value

        # IMU biases (used by simulator to generate noisy data)
        self.imu1_bias = np.array(self.get_parameter('imu1_bias').value)
        self.imu2_bias = np.array(self.get_parameter('imu2_bias').value)

        # Camera parameters
        self.camera_fov = np.radians(self.get_parameter('camera_fov_degrees').value)
        self.camera_fps = self.get_parameter('camera_fps').value
        self.camera_pixel_var = self.get_parameter('camera_pixel_variance').value
        self.camera_range = self.get_parameter('camera_range_max').value

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

        # Ghost vehicle states (from localization nodes)
        self.imu1_state = None
        self.imu2_state = None
        self.kalman_state = None

        # Initialize Pygame
        pygame.init()
        self.screen = pygame.display.set_mode((self.pool_width, self.pool_height))
        pygame.display.set_caption("2D Slalom Simulator")
        self.clock = pygame.time.Clock()
        self.font = pygame.font.Font(None, 20)

        # CV Bridge for image conversion
        self.bridge = CvBridge()

        # Publishers
        self.imu1_pub = self.create_publisher(Vector3Stamped, '/imu1/accel', 10)
        self.imu2_pub = self.create_publisher(Vector3Stamped, '/imu2/accel', 10)
        self.gt_pub = self.create_publisher(State, '/ground_truth', 10)
        self.camera_detections_pub = self.create_publisher(CameraDetections, '/camera/detections', 10)
        self.camera_frame_pub = self.create_publisher(Image, '/gamestate/frame', 10)
        self.force_pub = self.create_publisher(Vector3, '/control/force', 10)
        self.gates_pub = self.create_publisher(MarkerArray, '/slalom/gates', 10)

        # Subscribers
        self.control_sub = self.create_subscription(
            Vector3, '/control/command', self.control_callback, 10)
        self.imu1_state_sub = self.create_subscription(
            State, '/imu1/state', self.imu1_state_callback, 10)
        self.imu2_state_sub = self.create_subscription(
            State, '/imu2/state', self.imu2_state_callback, 10)
        self.kalman_state_sub = self.create_subscription(
            State, '/kalman/state', self.kalman_state_callback, 10)

        # Last acceleration for IMU (initialize BEFORE starting thread)
        self.last_accel = np.array([0.0, 0.0])
        self.last_angular_accel = 0.0

        # Timers
        self.create_timer(1.0 / 60.0, self.simulation_step)  # 60 Hz
        self.create_timer(1.0 / self.camera_fps, self.publish_camera_detections)  # Camera FPS
        self.create_timer(1.0 / 30.0, self.publish_camera_frame)  # 30 Hz for frame
        self.create_timer(1.0, self.publish_gates)            # 1 Hz

        # Start IMU publishing thread (after initializing last_accel)
        self.imu_running = True
        self.imu_thread = threading.Thread(target=self.imu_publishing_loop, daemon=True)
        self.imu_thread.start()

        self.get_logger().info('Simulator node initialized')
        self.publish_gates()  # Publish gates immediately

    def control_callback(self, msg):
        """Receive control force command"""
        self.control_force = np.array([msg.x, msg.y])

    def imu1_state_callback(self, msg):
        """Receive IMU1 state estimate for ghost rendering"""
        self.imu1_state = msg

    def imu2_state_callback(self, msg):
        """Receive IMU2 state estimate for ghost rendering"""
        self.imu2_state = msg

    def kalman_state_callback(self, msg):
        """Receive Kalman filter state estimate for ghost rendering"""
        self.kalman_state = msg

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
        """Update vehicle physics using semi-implicit Euler integration with process noise"""
        x, y, yaw, vx, vy, omega = self.state

        # Apply control force and damping
        ax = (self.control_force[0] - self.damping * vx) / self.mass
        ay = (self.control_force[1] - self.damping * vy) / self.mass
        alpha = -self.damping * omega  # Angular damping

        # Store for IMU (clean acceleration before noise)
        self.last_accel = np.array([ax, ay])
        self.last_angular_accel = alpha

        # Build process noise covariance matrix Q (6x6)
        # State: [x, y, yaw, vx, vy, omega]
        Q = build_Q_matrix(self.dt, self.sigma_ax2, self.sigma_ay2, self.sigma_alpha2)

        # Sample process noise from multivariate normal
        process_noise = np.random.multivariate_normal(np.zeros(6), Q)

        # Update velocities (with process noise on velocity components)
        vx += ax * self.dt + process_noise[3]
        vy += ay * self.dt + process_noise[4]
        omega += alpha * self.dt + process_noise[5]

        # Update positions (with process noise on position components)
        x += vx * self.dt + process_noise[0]
        y += vy * self.dt + process_noise[1]
        yaw += omega * self.dt + process_noise[2]
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

        # Draw ghost vehicles (localization estimates)
        self.draw_ghost(self.imu1_state, "IMU1")
        self.draw_ghost(self.imu2_state, "IMU2")
        self.draw_ghost(self.kalman_state, "KF")

        # Draw camera FOV cone
        self.draw_camera_fov()

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

    def draw_ghost(self, state_msg, label):
        """Draw a semi-transparent ghost vehicle from State message"""
        if state_msg is None:
            return

        x = state_msg.x
        y = state_msg.y
        yaw = state_msg.yaw

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

    def draw_camera_fov(self):
        """Draw camera field of view cone"""
        x, y, yaw, _, _, _ = self.state

        # Camera points in direction of vehicle yaw
        # FOV cone: yaw ± camera_fov/2
        fov_left = yaw - self.camera_fov / 2
        fov_right = yaw + self.camera_fov / 2

        # Draw triangle of light
        range_vis = min(self.camera_range, 150)  # Limit visual range for display
        left_point = np.array([
            x + range_vis * np.cos(fov_left),
            y + range_vis * np.sin(fov_left)
        ])
        right_point = np.array([
            x + range_vis * np.cos(fov_right),
            y + range_vis * np.sin(fov_right)
        ])

        # Draw semi-transparent cone
        cone_surface = pygame.Surface((self.pool_width, self.pool_height), pygame.SRCALPHA)
        pygame.draw.polygon(cone_surface, (255, 255, 0, 30),  # Yellow with low alpha
                          [(int(x), int(y)),
                           (int(left_point[0]), int(left_point[1])),
                           (int(right_point[0]), int(right_point[1]))])
        self.screen.blit(cone_surface, (0, 0))

        # Draw FOV boundary lines
        pygame.draw.line(self.screen, (255, 255, 0, 100),
                        (int(x), int(y)),
                        (int(left_point[0]), int(left_point[1])), 1)
        pygame.draw.line(self.screen, (255, 255, 0, 100),
                        (int(x), int(y)),
                        (int(right_point[0]), int(right_point[1])), 1)

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
        """Publish clean IMU acceleration data at 100 Hz in separate thread

        Note: Simulator publishes CLEAN acceleration (true dynamics).
        IMU localization nodes will add their own measurement noise + bias.
        """
        rate = 100.0  # Hz
        period = 1.0 / rate

        while self.imu_running and rclpy.ok():
            start_time = time.time()

            # Get true acceleration from physics (clean, no noise/bias)
            ax, ay = self.last_accel
            alpha = self.last_angular_accel

            # Publish same clean data to both IMUs
            # Each IMU node will add its own noise and bias
            timestamp = self.get_clock().now().to_msg()

            # Publish IMU1 acceleration
            imu1_msg = Vector3Stamped()
            imu1_msg.header = Header()
            imu1_msg.header.stamp = timestamp
            imu1_msg.header.frame_id = 'imu1_frame'
            imu1_msg.vector.x = float(ax)
            imu1_msg.vector.y = float(ay)
            imu1_msg.vector.z = float(alpha)
            self.imu1_pub.publish(imu1_msg)

            # Publish IMU2 acceleration
            imu2_msg = Vector3Stamped()
            imu2_msg.header = Header()
            imu2_msg.header.stamp = timestamp
            imu2_msg.header.frame_id = 'imu2_frame'
            imu2_msg.vector.x = float(ax)
            imu2_msg.vector.y = float(ay)
            imu2_msg.vector.z = float(alpha)
            self.imu2_pub.publish(imu2_msg)

            # Sleep to maintain rate
            elapsed = time.time() - start_time
            sleep_time = period - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    def publish_ground_truth(self):
        """Publish ground truth state"""
        msg = State()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'

        x, y, yaw, vx, vy, omega = self.state
        msg.x = float(x)
        msg.y = float(y)
        msg.yaw = float(yaw)
        msg.vx = float(vx)
        msg.vy = float(vy)
        msg.wyaw = float(omega)

        self.gt_pub.publish(msg)

    def publish_control_force(self):
        """Publish current control force"""
        msg = Vector3()
        msg.x = float(self.control_force[0])
        msg.y = float(self.control_force[1])
        msg.z = 0.0
        self.force_pub.publish(msg)

    def publish_camera_detections(self):
        """Publish pole detections within camera FOV"""
        x, y, yaw, _, _, _ = self.state

        # Camera orientation = vehicle yaw (fixed mounting)
        camera_angle = yaw
        fov_left = camera_angle - self.camera_fov / 2
        fov_right = camera_angle + self.camera_fov / 2

        detections = CameraDetections()
        detections.header = Header()
        detections.header.stamp = self.get_clock().now().to_msg()
        detections.header.frame_id = 'camera_frame'

        # Check each pole
        for gate in self.gates:
            for pole_type, pole_pos in [('red', gate['red_pole']), ('white', gate['white_pole'])]:
                # Vector from vehicle to pole
                dx = pole_pos[0] - x
                dy = pole_pos[1] - y
                distance = np.sqrt(dx**2 + dy**2)

                # Check range
                if distance > self.camera_range:
                    continue

                # Angle to pole
                angle_to_pole = np.arctan2(dy, dx)

                # Check if within FOV (handle angle wrapping)
                def angle_in_range(angle, left, right):
                    # Normalize all angles to [-pi, pi]
                    angle = normalize_angle(angle)
                    left = normalize_angle(left)
                    right = normalize_angle(right)

                    # Check if angle is between left and right
                    if left <= right:
                        return left <= angle <= right
                    else:  # Wraps around -pi/pi
                        return angle >= left or angle <= right

                if not angle_in_range(angle_to_pole, fov_left, fov_right):
                    continue

                # Pole is visible! Add pixel noise
                # Simplified model: add Gaussian noise to position
                noise_x = np.random.normal(0, np.sqrt(self.camera_pixel_var))
                noise_y = np.random.normal(0, np.sqrt(self.camera_pixel_var))

                detection = CameraDetection()
                detection.x = float(pole_pos[0] + noise_x)
                detection.y = float(pole_pos[1] + noise_y)
                detection.color = 0 if pole_type == 'red' else 1  # 0=red, 1=white/green
                detection.confidence = 1.0  # Could vary based on distance

                detections.detections.append(detection)

        self.camera_detections_pub.publish(detections)

    def publish_camera_frame(self):
        """Publish pygame surface as camera image frame"""
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
            self.camera_frame_pub.publish(img_msg)
        except Exception as e:
            self.get_logger().error(f'Failed to publish camera frame: {e}')

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
