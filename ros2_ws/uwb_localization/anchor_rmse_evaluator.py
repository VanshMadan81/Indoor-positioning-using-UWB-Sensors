import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class AnchorRMSEEvaluator(Node):

    def __init__(self):

        super().__init__('anchor_rmse_evaluator')

        self.subscription = self.create_subscription(
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

        self.evaluated = False

        self.get_logger().info("Anchor RMSE Evaluator Started")

    def anchor_callback(self, msg):

        if self.evaluated:
            return

        data = msg.data

        if len(data) != 18:
            self.get_logger().warn(
                f"Expected 18 values, received {len(data)}"
            )
            return

        estimated_anchors = np.array(data).reshape((6, 3))

        proper_rotation, proper_translation = self.kabsch_alignment(
            self.true_anchors,
            estimated_anchors,
            allow_reflection=False
        )

        reflected_rotation, reflected_translation = self.kabsch_alignment(
            self.true_anchors,
            estimated_anchors,
            allow_reflection=True
        )

        proper_aligned_anchors = (
            proper_rotation @ estimated_anchors.T
        ).T + proper_translation

        reflected_aligned_anchors = (
            reflected_rotation @ estimated_anchors.T
        ).T + reflected_translation

        proper_errors = np.linalg.norm(
            self.true_anchors - proper_aligned_anchors,
            axis=1
        )

        reflected_errors = np.linalg.norm(
            self.true_anchors - reflected_aligned_anchors,
            axis=1
        )

        proper_coordinate_rmse = np.sqrt(
            np.mean(proper_errors**2)
        )

        reflected_coordinate_rmse = np.sqrt(
            np.mean(reflected_errors**2)
        )

        self.print_alignment_comparison(
            proper_errors,
            proper_coordinate_rmse,
            reflected_errors,
            reflected_coordinate_rmse
        )

        self.get_logger().info(
            f"Proper Rotation Matrix =\n{proper_rotation}"
        )

        self.get_logger().info(
            f"Proper Translation Vector = {proper_translation}"
        )

        self.get_logger().info(
            f"Reflection-Allowed Rotation Matrix =\n{reflected_rotation}"
        )

        self.get_logger().info(
            f"Reflection-Allowed Translation Vector = {reflected_translation}"
        )

        self.evaluated = True

    def kabsch_alignment(self, true_anchors, estimated_anchors, allow_reflection):

        true_centroid = np.mean(
            true_anchors,
            axis=0
        )

        estimated_centroid = np.mean(
            estimated_anchors,
            axis=0
        )

        true_centered = true_anchors - true_centroid
        estimated_centered = estimated_anchors - estimated_centroid

        covariance = estimated_centered.T @ true_centered

        u, _, vt = np.linalg.svd(covariance)

        rotation = vt.T @ u.T

        if np.linalg.det(rotation) < 0.0 and not allow_reflection:
            vt[-1, :] *= -1.0
            rotation = vt.T @ u.T

        translation = true_centroid - rotation @ estimated_centroid

        return rotation, translation

    def print_alignment_comparison(
        self,
        proper_errors,
        proper_coordinate_rmse,
        reflected_errors,
        reflected_coordinate_rmse
    ):

        self.get_logger().info("Anchor RMSE Validation")
        self.get_logger().info("Proper Rotation Only")

        for i, error in enumerate(proper_errors):
            self.get_logger().info(
                f"A{i} Error = {error:.3f}"
            )

        self.get_logger().info(
            f"Coordinate RMSE = {proper_coordinate_rmse:.3f}"
        )

        self.get_logger().info("Reflection Allowed")

        for i, error in enumerate(reflected_errors):
            self.get_logger().info(
                f"A{i} Error = {error:.3f}"
            )

        self.get_logger().info(
            f"Coordinate RMSE = {reflected_coordinate_rmse:.3f}"
        )

        if reflected_coordinate_rmse < proper_coordinate_rmse:
            self.get_logger().info("Lower RMSE = Reflection Allowed")
        else:
            self.get_logger().info("Lower RMSE = Proper Rotation Only")


def main(args=None):

    rclpy.init(args=args)

    node = AnchorRMSEEvaluator()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
