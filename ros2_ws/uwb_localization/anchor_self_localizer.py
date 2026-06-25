import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

import numpy as np
from scipy.optimize import least_squares


class AnchorSelfLocalizer(Node):

    def __init__(self):

        super().__init__('anchor_self_localizer')

        self.subscription = self.create_subscription(
            Float64MultiArray,
            '/uwb_anchor_distances',
            self.anchor_distance_callback,
            10
        )

        self.publisher_ = self.create_publisher(
            Float64MultiArray,
            '/estimated_anchor_positions',
            10
        )

        self.timer = self.create_timer(
            0.5,
            self.publish_estimated_anchor_timer
        )

        self.anchor_pairs = [
            (0, 1), (0, 2), (0, 3), (0, 4), (0, 5),
            (1, 2), (1, 3), (1, 4), (1, 5),
            (2, 3), (2, 4), (2, 5),
            (3, 4), (3, 5),
            (4, 5)
        ]

        self.distance_sums = {
            pair: 0.0 for pair in self.anchor_pairs
        }

        self.distance_counts = {
            pair: 0 for pair in self.anchor_pairs
        }

        self.message_count = 0
        self.state = 'COLLECTING'
        self.estimated_anchors = None

        self.get_logger().info("Anchor Self Localizer Started")

    def anchor_distance_callback(self, msg):

        if self.state == 'DONE':
            return

        data = msg.data

        if len(data) != 45:
            self.get_logger().warn(
                f"Expected 45 values, received {len(data)}"
            )
            return

        for k in range(0, len(data), 3):
            i = int(data[k])
            j = int(data[k + 1])
            distance = data[k + 2]

            pair = (
                min(i, j),
                max(i, j)
            )

            if pair not in self.distance_sums:
                self.get_logger().warn(
                    f"Ignoring invalid anchor pair {pair}"
                )
                continue

            self.distance_sums[pair] += distance
            self.distance_counts[pair] += 1

        self.message_count += 1

        if self.message_count >= 100:
            self.print_average_distance_table()
            self.estimate_anchor_coordinates()
            self.state = 'DONE'

    def compute_average_distances(self):

        average_distances = {}

        for pair in self.anchor_pairs:
            count = self.distance_counts[pair]

            if count == 0:
                average_distances[pair] = 0.0
            else:
                average_distances[pair] = self.distance_sums[pair] / count

        return average_distances

    def print_average_distance_table(self):

        average_distances = self.compute_average_distances()

        self.get_logger().info("Pair    Average Distance (m)")

        for pair in self.anchor_pairs:
            self.get_logger().info(
                f"{pair[0]}-{pair[1]}     {average_distances[pair]:.3f}"
            )

    def estimate_anchor_coordinates(self):

        average_distances = self.compute_average_distances()

        d01 = average_distances[(0, 1)]
        d02 = average_distances[(0, 2)]
        d12 = average_distances[(1, 2)]

        x2 = (
            d01**2 + d02**2 - d12**2
        ) / (2.0 * d01)

        y2_squared = d02**2 - x2**2

        if y2_squared < 0.0:
            y2_squared = 0.0

        y2 = np.sqrt(y2_squared)

        initial_guess = np.array([
            x2,
            y2,
            6.0,
            2.0,
            2.0,
            0.0,
            6.0,
            2.0,
            6.0,
            6.0,
            2.0
        ])

        result = least_squares(
            self.residuals,
            initial_guess,
            args=(average_distances,),
            method='trf'
        )

        anchors = self.reconstruct_anchors(
            result.x,
            d01
        )

        self.print_estimated_anchors(anchors)
        self.print_distance_validation(
            anchors,
            average_distances
        )
        self.estimated_anchors = anchors

    def reconstruct_anchors(self, state, d01):

        return np.array([
            [0.0, 0.0, 0.0],
            [d01, 0.0, 0.0],
            [state[0], state[1], 0.0],
            [state[2], state[3], state[4]],
            [state[5], state[6], state[7]],
            [state[8], state[9], state[10]]
        ])

    def residuals(self, state, average_distances):

        anchors = self.reconstruct_anchors(
            state,
            average_distances[(0, 1)]
        )

        residuals = []

        for pair in self.anchor_pairs:
            i, j = pair

            predicted_distance = np.linalg.norm(
                anchors[i] - anchors[j]
            )

            residuals.append(
                predicted_distance - average_distances[pair]
            )

        return np.array(residuals)

    def print_estimated_anchors(self, anchors):

        self.get_logger().info("Estimated Anchor Coordinates")

        for i, anchor in enumerate(anchors):
            self.get_logger().info(
                f"A{i} = [{anchor[0]:.3f}, {anchor[1]:.3f}, {anchor[2]:.3f}]"
            )

    def print_distance_validation(self, anchors, average_distances):

        distance_errors = []

        for pair in self.anchor_pairs:
            i, j = pair

            estimated_distance = np.linalg.norm(
                anchors[i] - anchors[j]
            )

            distance_error = (
                estimated_distance - average_distances[pair]
            )

            distance_errors.append(distance_error)

        distance_errors = np.array(distance_errors)

        mean_absolute_error = np.mean(np.abs(distance_errors))
        max_error = np.max(np.abs(distance_errors))
        rmse = np.sqrt(np.mean(distance_errors**2))

        self.get_logger().info("Distance Validation")
        self.get_logger().info(f"Mean Error = {mean_absolute_error:.3f}")
        self.get_logger().info(f"Max Error = {max_error:.3f}")
        self.get_logger().info(f"RMSE = {rmse:.3f}")

    def publish_estimated_anchor_timer(self):

        if self.estimated_anchors is not None:
            self.publish_estimated_anchors(self.estimated_anchors)

    def publish_estimated_anchors(self, anchors):

        msg = Float64MultiArray()
        msg.data = anchors.flatten().tolist()

        self.publisher_.publish(msg)
        self.get_logger().info("Published estimated anchor positions")


def main(args=None):

    rclpy.init(args=args)

    node = AnchorSelfLocalizer()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
