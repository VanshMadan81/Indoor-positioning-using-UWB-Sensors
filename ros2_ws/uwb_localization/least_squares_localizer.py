import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Float64MultiArray

import numpy as np
from scipy.optimize import least_squares


class LeastSquaresLocalizer(Node):

    def __init__(self):

        super().__init__('least_squares_localizer')

        self.subscription = self.create_subscription(
            Float64MultiArray,
            '/uwb_distances',
            self.distance_callback,
            10
        )

        self.anchor_subscription = self.create_subscription(
            Float64MultiArray,
            '/estimated_anchor_positions',
            self.anchor_callback,
            10
        )

        self.anchors = np.array([
            [-3.0, -3.0, 1.0],
            [ 3.0, -3.0, 2.0],
            [-3.0,  0.0, 3.0],
            [ 3.0,  0.0, 1.5],
            [-3.0,  3.0, 2.5],
            [ 3.0,  3.0, 3.5]
        ])

        self.asl_anchors_ready = False

        self.get_logger().info(
            "Least Squares Localizer Started"
        )
        self.publisher_ = self.create_publisher(
            Point,
            '/ls_position',
            10
        )

    def anchor_callback(self, msg):

        if len(msg.data) != 18:
            self.get_logger().warn(
                f"Expected 18 anchor values, received {len(msg.data)}"
            )
            return

        self.anchors = np.array(msg.data).reshape((6, 3))
        self.asl_anchors_ready = True

        self.get_logger().info(
            "Updated anchors from /estimated_anchor_positions"
        )

    def residuals(self, position, anchors, distances):

        x, y, z = position

        predicted = np.sqrt(
            (anchors[:,0] - x)**2 +
            (anchors[:,1] - y)**2 +
            (anchors[:,2] - z)**2
        )

        return predicted - distances

    def distance_callback(self, msg):

        if not self.asl_anchors_ready:
            return

        distances = np.array(msg.data)

        initial_guess = np.array([0.0, 0.0, 0.0])

        result = least_squares(
            self.residuals,
            initial_guess,
            args=(self.anchors, distances)
        )

        x_est = result.x[0]
        y_est = result.x[1]
        z_est = result.x[2]
        point = Point()

        point.x = float(x_est)
        point.y = float(y_est)
        point.z = float(z_est)

        self.publisher_.publish(point)
        self.get_logger().info(
            f"Estimated Position = ({x_est:.2f}, {y_est:.2f}, {z_est:.2f})"
        )


def main(args=None):

    rclpy.init(args=args)

    node = LeastSquaresLocalizer()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
