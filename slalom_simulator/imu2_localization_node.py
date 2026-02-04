#!/usr/bin/env python3
"""
IMU2 Localization Node - Dead reckoning using IMU2 data.
Integrates IMU2 accelerations to estimate vehicle position.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
import numpy as np


class IMU2LocalizationNode(Node):
    def __init__(self):
        super().__init__('imu2_localization_node')

        # Declare and get parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('vehicle_start_x', 400.0),
                ('vehicle_start_y', 500.0),
            ]
        )

        # Initialize state [x, y, yaw, vx, vy, omega]
        start_x = 400 #self.get_parameter('vehicle_start_x').value
        start_y = 100 #self.get_parameter('vehicle_start_y').value
        self.state = np.array([start_x, start_y, 0.0, 0.0, 0.0, 0.0])

        # Last timestamp for integration
        self.last_time = None

        # Publisher
        self.pose_pub = self.create_publisher(PoseStamped, '/imu2/pose', 10)

        # Subscriber
        self.imu_sub = self.create_subscription(
            Imu, '/imu2/data', self.imu_callback, 10)

        self.get_logger().info('IMU2 Localization node initialized')

    def imu_callback(self, msg):
        """Process IMU2 data and integrate to estimate position"""
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        if self.last_time is None:
            self.last_time = current_time
            return

        dt = current_time - self.last_time
        if dt <= 0 or dt > 0.1:  # Sanity check
            self.last_time = current_time
            return

        # Extract measurements
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        omega = msg.angular_velocity.z

        # Current state
        x, y, yaw, vx, vy, old_omega = self.state

        # Update velocities using measured accelerations
        vx += ax * dt
        vy += ay * dt

        # Update position using velocities
        x += vx * dt
        y += vy * dt

        # Update yaw using angular velocity
        yaw += omega * dt

        # Normalize yaw to [-pi, pi]
        while yaw > np.pi:
            yaw -= 2 * np.pi
        while yaw < -np.pi:
            yaw += 2 * np.pi

        # Store updated state
        self.state = np.array([x, y, yaw, vx, vy, omega])
        self.last_time = current_time

        # Publish pose estimate
        self.publish_pose(msg.header.stamp)

    def publish_pose(self, timestamp):
        """Publish estimated pose"""
        msg = PoseStamped()
        msg.header = Header()
        msg.header.stamp = timestamp
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

        self.pose_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = IMU2LocalizationNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
