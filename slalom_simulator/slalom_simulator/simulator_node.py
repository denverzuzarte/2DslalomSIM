#!/usr/bin/env python3
"""
Main Pygame-based simulator node for AUV 3D navigation.
Simulates 3D vehicle physics; display remains 2D top-down view.

Z-axis convention (sim units / cm):
  z = 0  => surface
  z > 0  => submerged (positive depth)
  Buoyancy provides a constant upward acceleration (buoyancy_accel < 0)
  only when z > 0. At the surface (z <= 0) the vehicle is clamped.

Topics removed:
  /control/command  — was internal bridge between controller and simulator
  /control/force    — was echo of applied force; redundant

Topics changed:
  /imu1/accel, /imu2/accel — now ImuAccel (combined linear + angular)
  /controller/force         — now ImuAccel (6D: acc + ang_acc command)
  /controller/setpoint      — AuvState (full state setpoint)
"""
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
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

from auv_msgs.msg import AuvState, Orientation, ObjectDetections, ObjectDetected, ImuAccel


class SimulatorNode(Node):
    def __init__(self):
        super().__init__('simulator_node')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('pool_width', 600),
                ('pool_height', 800),
                ('vehicle_size', 40),
                ('vehicle_start_x', 400.0),
                ('vehicle_start_y', 500.0),
                ('vehicle_start_z', 0.0),
                # Process noise (3D + yaw) — reduced ~60% from old defaults
                ('process_noise.sigma_ax_squared', 0.004),
                ('process_noise.sigma_ay_squared', 0.004),
                ('process_noise.sigma_az_squared', 0.002),
                ('process_noise.sigma_alpha_squared', 0.0004),
                # Buoyancy: upward accel when submerged (negative => upward in +Z convention)
                ('buoyancy_accel', -0.10),
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

        self.pool_width = self.get_parameter('pool_width').value
        self.pool_height = self.get_parameter('pool_height').value
        self.vehicle_size = self.get_parameter('vehicle_size').value

        # 3D AUV state: [x, y, z, yaw, vx, vy, vz, wyaw]
        self.state = np.array([
            self.get_parameter('vehicle_start_x').value,
            self.get_parameter('vehicle_start_y').value,
            self.get_parameter('vehicle_start_z').value,
            0.0,  # yaw
            0.0,  # vx
            0.0,  # vy
            0.0,  # vz
            0.0,  # wyaw
        ])

        self.sigma_ax2 = self.get_parameter('process_noise.sigma_ax_squared').value
        self.sigma_ay2 = self.get_parameter('process_noise.sigma_ay_squared').value
        self.sigma_az2 = self.get_parameter('process_noise.sigma_az_squared').value
        self.sigma_alpha2 = self.get_parameter('process_noise.sigma_alpha_squared').value
        self.buoyancy_accel = self.get_parameter('buoyancy_accel').value

        self.camera_fov = np.radians(self.get_parameter('camera_fov_degrees').value)
        self.camera_fps = self.get_parameter('camera_fps').value
        self.camera_pixel_var = self.get_parameter('camera_pixel_variance').value
        self.camera_range = self.get_parameter('camera_range_max').value

        self.mass = 1.0
        self.damping = 0.1
        self.dt = 1.0 / 60.0  # 60 Hz

        # Applied force [fx, fy, fz] from /controller/force (6D ImuAccel — lin part)
        self.control_force = np.array([0.0, 0.0, 0.0])

        self.gates = generate_gates(
            num_gates=self.get_parameter('num_gates').value,
            pool_width=self.pool_width,
            pool_height=self.pool_height,
            gate_width_mean=self.get_parameter('gate_width_mean').value,
            gate_width_std=self.get_parameter('gate_width_std').value,
            pole_separation_mean=self.get_parameter('pole_separation_mean').value,
            pole_separation_std=self.get_parameter('pole_separation_std').value
        )

        # Ghost states from localization nodes (for Pygame overlay)
        self.imu1_state = None
        self.imu2_state = None

        # Pygame (2D top-down view)
        pygame.init()
        self.screen = pygame.display.set_mode((self.pool_width, self.pool_height))
        pygame.display.set_caption("AUV 3D Simulator (Top-Down View)")
        self.clock = pygame.time.Clock()
        self.font = pygame.font.Font(None, 20)

        self.bridge = CvBridge()

        # --- Publishers ---
        # Combined IMU accel (linear + angular) for each IMU
        self.imu1_pub = self.create_publisher(ImuAccel, '/imu1/accel', 10)
        self.imu2_pub = self.create_publisher(ImuAccel, '/imu2/accel', 10)
        # Ground truth & detections
        self.gt_pub = self.create_publisher(AuvState, '/ground_truth', 10)
        self.camera_detections_pub = self.create_publisher(ObjectDetections, '/camera/detections', 10)
        self.camera_frame_pub = self.create_publisher(Image, '/auv/frame', 10)
        self.gates_pub = self.create_publisher(MarkerArray, '/auv/gates', 10)

        # --- Subscribers ---
        # /controller/force: 6D acceleration command (ImuAccel) — lin.xyz used as [fx,fy,fz]
        self.force_sub = self.create_subscription(
            ImuAccel, '/controller/force', self.force_callback, 10)
        # State overlays for visualization
        self.imu1_state_sub = self.create_subscription(
            AuvState, '/imu1/state', self.imu1_state_callback, 10)
        self.imu2_state_sub = self.create_subscription(
            AuvState, '/imu2/state', self.imu2_state_callback, 10)

        # Clean accelerations for IMU thread: [ax, ay, az, alpha]
        self.last_accel = np.array([0.0, 0.0, 0.0, 0.0])

        # Timers
        self.create_timer(1.0 / 60.0, self.simulation_step)
        self.create_timer(1.0 / self.camera_fps, self.publish_camera_detections)
        self.create_timer(1.0 / 30.0, self.publish_camera_frame)
        self.create_timer(1.0, self.publish_gates)

        self.imu_running = True
        self.imu_thread = threading.Thread(target=self.imu_publishing_loop, daemon=True)
        self.imu_thread.start()

        self.get_logger().info('AUV Simulator node initialized (3D physics, 2D display)')
        self.publish_gates()

    def force_callback(self, msg: ImuAccel):
        """Receive 6D force/accel command; use linear part as [fx, fy, fz]."""
        self.control_force = np.array([
            msg.linear.x, msg.linear.y, msg.linear.z
        ])

    def imu1_state_callback(self, msg: AuvState):
        self.imu1_state = msg

    def imu2_state_callback(self, msg: AuvState):
        self.imu2_state = msg

    def simulation_step(self):
        """Main simulation step at 60 Hz"""
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.imu_running = False
                rclpy.shutdown()
                return

        self.update_dynamics()
        self.render()
        self.publish_ground_truth()
        self.clock.tick(60)

    def update_dynamics(self):
        """
        Update 3D vehicle physics — semi-implicit Euler + process noise.

        State: [x, y, z, yaw, vx, vy, vz, wyaw]

        Buoyancy: constant upward acceleration (buoyancy_accel < 0 in +Z-down convention)
        applied only when z > 0 (submerged). At z <= 0 vehicle is at surface.
        """
        x, y, z, yaw, vx, vy, vz, wyaw = self.state

        # XY accelerations: control + damping
        ax = (self.control_force[0] - self.damping * vx) / self.mass
        ay = (self.control_force[1] - self.damping * vy) / self.mass

        # Z acceleration: control + damping + buoyancy (when submerged)
        az_buoy = self.buoyancy_accel if z > 0.0 else 0.0
        az = (self.control_force[2] - self.damping * vz) / self.mass + az_buoy

        # Angular: damping only
        alpha = -self.damping * wyaw

        # Store clean accelerations for IMU publishing [ax, ay, az, alpha]
        self.last_accel = np.array([ax, ay, az, alpha])

        # 8D process noise
        Q = build_Q_matrix(self.dt, self.sigma_ax2, self.sigma_ay2,
                           self.sigma_az2, self.sigma_alpha2)
        pn = np.random.multivariate_normal(np.zeros(8), Q)

        # Velocity update
        vx += ax * self.dt + pn[4]
        vy += ay * self.dt + pn[5]
        vz += az * self.dt + pn[6]
        wyaw += alpha * self.dt + pn[7]

        # Position update
        x += vx * self.dt + pn[0]
        y += vy * self.dt + pn[1]
        z += vz * self.dt + pn[2]
        yaw += wyaw * self.dt + pn[3]
        yaw = normalize_angle(yaw)

        # Z floor: clamp to surface
        if z <= 0.0:
            z = 0.0
            if vz < 0.0:
                vz = 0.0

        # XY boundary (bounce)
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

        self.state = np.array([x, y, z, yaw, vx, vy, vz, wyaw])

    def render(self):
        """Render 2D top-down view using Pygame"""
        self.screen.fill(COLOR_BACKGROUND)

        # Draw gates
        for gate in self.gates:
            red_pos = gate['red_pole'].astype(int)
            white_pos = gate['white_pole'].astype(int)
            pygame.draw.circle(self.screen, COLOR_RED_POLE, red_pos, 4)
            pygame.draw.circle(self.screen, COLOR_WHITE_POLE, white_pos, 4)

        # Ghost vehicles
        self.draw_ghost(self.imu1_state, "IMU1")
        self.draw_ghost(self.imu2_state, "IMU2")

        # Camera FOV cone
        self.draw_camera_fov()

        # Main vehicle
        x, y, z, yaw, _, _, _, _ = self.state
        half_size = self.vehicle_size / 2

        corners = np.array([
            [-half_size, -half_size],
            [half_size, -half_size],
            [half_size, half_size],
            [-half_size, half_size]
        ])
        cos_yaw = np.cos(yaw)
        sin_yaw = np.sin(yaw)
        rotation = np.array([[cos_yaw, -sin_yaw], [sin_yaw, cos_yaw]])
        rotated_corners = (rotation @ corners.T).T + np.array([x, y])
        pygame.draw.polygon(self.screen, COLOR_VEHICLE, rotated_corners.astype(int))

        # Force arrow
        if np.linalg.norm(self.control_force[:2]) > 0.1:
            self.draw_arrow(x, y, self.control_force[:2])

        # HUD: Z depth indicator
        z_text = self.font.render(f"Z: {z:.2f} cm", True, (0, 0, 0))
        self.screen.blit(z_text, (10, 10))

        pygame.display.flip()

    def draw_ghost(self, state_msg, label):
        """Draw a semi-transparent ghost vehicle from AuvState message"""
        if state_msg is None:
            return
        x = state_msg.position.x
        y = state_msg.position.y
        yaw = state_msg.orientation.yaw

        half_size = self.vehicle_size / 2
        corners = np.array([
            [-half_size, -half_size],
            [half_size, -half_size],
            [half_size, half_size],
            [-half_size, half_size]
        ])
        cos_yaw = np.cos(yaw)
        sin_yaw = np.sin(yaw)
        rotation = np.array([[cos_yaw, -sin_yaw], [sin_yaw, cos_yaw]])
        rotated_corners = (rotation @ corners.T).T + np.array([x, y])

        ghost_surface = pygame.Surface((self.pool_width, self.pool_height), pygame.SRCALPHA)
        pygame.draw.polygon(ghost_surface, (*COLOR_GHOST, 128), rotated_corners.astype(int))
        self.screen.blit(ghost_surface, (0, 0))

        text = self.font.render(label, True, (0, 0, 0))
        self.screen.blit(text, (int(x) - 15, int(y) - half_size - 20))

    def draw_camera_fov(self):
        """Draw camera FOV cone"""
        x, y, z, yaw, _, _, _, _ = self.state
        fov_left = yaw - self.camera_fov / 2
        fov_right = yaw + self.camera_fov / 2

        range_vis = min(self.camera_range, 150)
        left_point = np.array([
            x + range_vis * np.cos(fov_left),
            y + range_vis * np.sin(fov_left)
        ])
        right_point = np.array([
            x + range_vis * np.cos(fov_right),
            y + range_vis * np.sin(fov_right)
        ])

        cone_surface = pygame.Surface((self.pool_width, self.pool_height), pygame.SRCALPHA)
        pygame.draw.polygon(cone_surface, (255, 255, 0, 30),
                            [(int(x), int(y)),
                             (int(left_point[0]), int(left_point[1])),
                             (int(right_point[0]), int(right_point[1]))])
        self.screen.blit(cone_surface, (0, 0))
        pygame.draw.line(self.screen, (255, 255, 0, 100),
                         (int(x), int(y)),
                         (int(left_point[0]), int(left_point[1])), 1)
        pygame.draw.line(self.screen, (255, 255, 0, 100),
                         (int(x), int(y)),
                         (int(right_point[0]), int(right_point[1])), 1)

    def draw_arrow(self, x, y, force):
        """Draw force arrow from vehicle center"""
        scale = 5.0
        arrow_end = np.array([x, y]) + force * scale
        pygame.draw.line(self.screen, COLOR_FORCE_ARROW,
                         (int(x), int(y)),
                         (int(arrow_end[0]), int(arrow_end[1])), 3)
        angle = np.arctan2(force[1], force[0])
        arrow_size = 10
        left_angle = angle + 2.5
        right_angle = angle - 2.5
        left_point = arrow_end + arrow_size * np.array([np.cos(left_angle), np.sin(left_angle)])
        right_point = arrow_end + arrow_size * np.array([np.cos(right_angle), np.sin(right_angle)])
        pygame.draw.polygon(self.screen, COLOR_FORCE_ARROW,
                            [arrow_end.astype(int), left_point.astype(int), right_point.astype(int)])

    def imu_publishing_loop(self):
        """
        Publish clean 3D IMU acceleration data at 100 Hz as ImuAccel.
          linear  = [ax, ay, az]
          angular = [alpha, 0, 0]   (only yaw-axis angular accel modelled)
        """
        rate = 100.0
        period = 1.0 / rate

        while self.imu_running and rclpy.ok():
            start_time = time.time()
            ax, ay, az, alpha = self.last_accel
            timestamp = self.get_clock().now().to_msg()

            for imu_name, pub in [
                ('imu1', self.imu1_pub),
                ('imu2', self.imu2_pub),
            ]:
                msg = ImuAccel()
                msg.header.stamp = timestamp
                msg.header.frame_id = f'{imu_name}_frame'
                msg.linear.x = float(ax)
                msg.linear.y = float(ay)
                msg.linear.z = float(az)
                msg.angular.x = float(alpha)   # yaw angular accel
                msg.angular.y = 0.0
                msg.angular.z = 0.0
                pub.publish(msg)

            elapsed = time.time() - start_time
            sleep_time = period - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    def publish_ground_truth(self):
        """Publish 3D ground truth as AuvState"""
        x, y, z, yaw, vx, vy, vz, wyaw = self.state
        msg = AuvState()
        msg.header.stamp = self.get_clock().now().to_msg()
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
        ax, ay, az, _ = self.last_accel
        msg.acceleration.x = float(ax)
        msg.acceleration.y = float(ay)
        msg.acceleration.z = float(az)
        self.gt_pub.publish(msg)

    def publish_camera_detections(self):
        """Publish visible pole detections as ObjectDetections"""
        x, y, z, yaw, _, _, _, _ = self.state
        camera_angle = yaw
        fov_left = camera_angle - self.camera_fov / 2
        fov_right = camera_angle + self.camera_fov / 2

        detections = ObjectDetections()

        for gate in self.gates:
            for pole_name, pole_pos in [('red_pole', gate['red_pole']),
                                        ('white_pole', gate['white_pole'])]:
                dx = pole_pos[0] - x
                dy = pole_pos[1] - y
                dist = np.sqrt(dx ** 2 + dy ** 2)
                if dist > self.camera_range:
                    continue

                angle_to_pole = np.arctan2(dy, dx)

                def angle_in_range(angle, left, right):
                    angle = normalize_angle(angle)
                    left = normalize_angle(left)
                    right = normalize_angle(right)
                    if left <= right:
                        return left <= angle <= right
                    else:
                        return angle >= left or angle <= right

                if not angle_in_range(angle_to_pole, fov_left, fov_right):
                    continue

                noise_x = np.random.normal(0, np.sqrt(self.camera_pixel_var))
                noise_y = np.random.normal(0, np.sqrt(self.camera_pixel_var))

                det = ObjectDetected()
                det.name = pole_name
                det.glob_midpoint.x = float(pole_pos[0] + noise_x)
                det.glob_midpoint.y = float(pole_pos[1] + noise_y)
                det.glob_midpoint.z = 0.0
                detections.objects.append(det)

        self.camera_detections_pub.publish(detections)

    def publish_camera_frame(self):
        """Publish pygame surface as ROS Image"""
        frame = pygame.surfarray.array3d(self.screen)
        frame = np.transpose(frame, (1, 0, 2))
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        try:
            img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            img_msg.header.stamp = self.get_clock().now().to_msg()
            img_msg.header.frame_id = 'camera_frame'
            self.camera_frame_pub.publish(img_msg)
        except Exception as e:
            self.get_logger().error(f'Failed to publish camera frame: {e}')

    def publish_gates(self):
        """Publish gate markers"""
        marker_array = MarkerArray()
        for i, gate in enumerate(self.gates):
            for j, (color_name, pole_pos, color_rgba) in enumerate([
                ('red', gate['red_pole'], (1.0, 0.0, 0.0, 1.0)),
                ('white', gate['white_pole'], (1.0, 1.0, 1.0, 1.0)),
            ]):
                m = Marker()
                m.header.stamp = self.get_clock().now().to_msg()
                m.header.frame_id = 'world'
                m.ns = 'gates'
                m.id = i * 2 + j
                m.type = Marker.SPHERE
                m.action = Marker.ADD
                m.pose.position.x = float(pole_pos[0])
                m.pose.position.y = float(pole_pos[1])
                m.pose.position.z = 0.0
                m.pose.orientation.w = 1.0
                m.scale.x = 8.0
                m.scale.y = 8.0
                m.scale.z = 8.0
                m.color.r, m.color.g, m.color.b, m.color.a = color_rgba
                marker_array.markers.append(m)
        self.gates_pub.publish(marker_array)

    def destroy_node(self):
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
