import math

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class DiagnosticRangeValidator(Node):

    def __init__(self):

        super().__init__('diagnostic_range_validator')

        self.create_subscription(
            Float64MultiArray,
            '/uwb_distances',
            self.range_callback,
            10
        )

        self.create_subscription(
            Float64MultiArray,
            '/estimated_anchor_positions',
            self.anchor_callback,
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

        self.measured_ranges = None
        self.anchors = None
        self.ls_position = None
        self.ekf_position = None

        self.timer = self.create_timer(
            1.0,
            self.print_validation
        )

        self.get_logger().info("Diagnostic Range Validator Started")

    def range_callback(self, msg):

        if len(msg.data) != 6:
            self.get_logger().warn(
                f"Expected 6 UWB ranges, received {len(msg.data)}"
            )
            return

        self.measured_ranges = np.array(msg.data)

    def anchor_callback(self, msg):

        if len(msg.data) != 18:
            self.get_logger().warn(
                f"Expected 18 anchor values, received {len(msg.data)}"
            )
            return

        self.anchors = np.array(msg.data).reshape((6, 3))

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

    def print_validation(self):

        if not self.all_inputs_available():
            return

        self.print_position_validation(
            "LS",
            self.ls_position
        )

        self.print_position_validation(
            "EKF",
            self.ekf_position
        )

    def all_inputs_available(self):

        return (
            self.measured_ranges is not None and
            self.anchors is not None and
            self.ls_position is not None and
            self.ekf_position is not None
        )

    def print_position_validation(self, label, position):

        residuals = []

        self.get_logger().info("--------------------------------")
        self.get_logger().info(f"{label} RANGE VALIDATION")

        for i, anchor in enumerate(self.anchors):
            predicted_distance = math.sqrt(
                (anchor[0] - position[0])**2 +
                (anchor[1] - position[1])**2 +
                (anchor[2] - position[2])**2
            )

            measured_distance = self.measured_ranges[i]
            residual = predicted_distance - measured_distance
            residuals.append(residual)

            self.get_logger().info(f"A{i}:")
            self.get_logger().info(f"Measured = {measured_distance:.3f}")
            self.get_logger().info(f"Predicted = {predicted_distance:.3f}")
            self.get_logger().info(f"Residual = {residual:.3f}")

        residuals = np.array(residuals)
        rmse = math.sqrt(np.mean(residuals**2))

        self.get_logger().info(f"RMSE_{label} = {rmse:.3f}")
        self.get_logger().info("--------------------------------")


def main(args=None):

    rclpy.init(args=args)

    node = DiagnosticRangeValidator()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
