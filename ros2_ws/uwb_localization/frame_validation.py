import math

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class FrameValidation(Node):

    def __init__(self):

        super().__init__('frame_validation')

        self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.create_subscription(
            Point,
            '/ls_position',
            self.ls_callback,
            10
        )

        self.create_subscription(
            Point,
            '/ekf_position',
            self.ekf_callback,
            10
        )

        self.create_subscription(
            Float64MultiArray,
            '/estimated_anchor_positions',
            self.anchor_callback,
            10
        )

        self.true_anchors = np.array([
            [-3.0, -3.0, 1.0],
            [3.0, -3.0, 2.0],
            [-3.0, 0.0, 3.0],
            [3.0, 0.0, 1.5],
            [-3.0, 3.0, 2.5],
            [3.0, 3.0, 3.5]
        ])

        self.world_truth = None
        self.asl_truth = None
        self.ls_position = None
        self.ekf_position = None
        self.rotation_world_to_asl = None
        self.translation_world_to_asl = None
        self.asl_transform_ready = False

        self.timer = self.create_timer(
            1.0,
            self.print_validation
        )

        self.get_logger().info("Frame Validation Started")

    def anchor_callback(self, msg):

        if len(msg.data) != 18:
            self.get_logger().warn(
                f"Expected 18 anchor values, received {len(msg.data)}"
            )
            return

        estimated_anchors = np.array(msg.data).reshape((6, 3))

        self.rotation_world_to_asl, self.translation_world_to_asl = (
            self.kabsch_alignment(
                estimated_anchors,
                self.true_anchors
            )
        )

        self.get_logger().info(
            f"det(R) = {np.linalg.det(self.rotation_world_to_asl):.6f}"
        )

        self.get_logger().info(
            f"Rotation Matrix =\n{self.rotation_world_to_asl}"
        )

        self.asl_transform_ready = True

    def odom_callback(self, msg):

        self.world_truth = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ])

        if not self.asl_transform_ready:
            return

        self.asl_truth = (
            self.rotation_world_to_asl @ self.world_truth +
            self.translation_world_to_asl
        )

    def ls_callback(self, msg):

        self.ls_position = np.array([
            msg.x,
            msg.y,
            msg.z
        ])

    def ekf_callback(self, msg):

        self.ekf_position = np.array([
            msg.x,
            msg.y,
            msg.z
        ])

    def kabsch_alignment(self, target_anchors, source_anchors):

        target_centroid = np.mean(target_anchors, axis=0)
        source_centroid = np.mean(source_anchors, axis=0)

        target_centered = target_anchors - target_centroid
        source_centered = source_anchors - source_centroid

        covariance = source_centered.T @ target_centered

        u, _, vt = np.linalg.svd(covariance)
        rotation = vt.T @ u.T

        translation = target_centroid - rotation @ source_centroid

        return rotation, translation

    def print_validation(self):

        world_truth = self.format_vector(self.world_truth)
        asl_truth = self.format_vector(self.asl_truth)
        ls_position = self.format_vector(self.ls_position)
        ekf_position = self.format_vector(self.ekf_position)

        ls_error = self.compute_error(
            self.asl_truth,
            self.ls_position
        )

        ekf_error = self.compute_error(
            self.asl_truth,
            self.ekf_position
        )

        self.get_logger().info(
            f"World Truth = {world_truth}"
        )
        self.get_logger().info(
            f"ASL Truth = {asl_truth}"
        )
        self.get_logger().info(
            f"LS Position = {ls_position}"
        )
        self.get_logger().info(
            f"EKF Position = {ekf_position}"
        )
        self.get_logger().info(
            f"LS Error = {ls_error}"
        )
        self.get_logger().info(
            f"EKF Error = {ekf_error}"
        )

    def format_vector(self, vector):

        if vector is None:
            return "(not available)"

        return (
            f"({vector[0]:.3f}, "
            f"{vector[1]:.3f}, "
            f"{vector[2]:.3f})"
        )

    def compute_error(self, truth, position):

        if truth is None or position is None:
            return "not available"

        error = math.sqrt(
            (truth[0] - position[0])**2 +
            (truth[1] - position[1])**2 +
            (truth[2] - position[2])**2
        )

        return f"{error:.3f} m"


def main(args=None):

    rclpy.init(args=args)

    node = FrameValidation()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
