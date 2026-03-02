#!/usr/bin/env python3
"""
Ground Truth Localization Node — passes simulator ground truth directly to /localization/pose.

Use this when you want perfect localization (no IMU noise or drift).
Subscribes to /ground_truth (AuvState from simulator) and republishes to /localization/pose.

Topics:
  SUB /ground_truth      (AuvState) — perfect state from simulator
  PUB /localization/pose (AuvState) — forwarded as localization output
"""
import rclpy
from rclpy.node import Node

from auv_msgs.msg import AuvState


class GroundTruthLocalizationNode(Node):
    def __init__(self):
        super().__init__('ground_truth_localization_node')

        self.pose_pub = self.create_publisher(AuvState, '/localization/pose', 10)

        self.gt_sub = self.create_subscription(
            AuvState, '/ground_truth', self.gt_callback, 10)

        self.get_logger().info(
            'Ground truth localization node initialized — '
            'forwarding /ground_truth -> /localization/pose'
        )

    def gt_callback(self, msg: AuvState):
        self.pose_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GroundTruthLocalizationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
