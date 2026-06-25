import math
import random

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class AnchorDistanceSimulator(Node):

    def __init__(self):

        super().__init__('anchor_distance_simulator')

        self.publisher_ = self.create_publisher(
            Float64MultiArray,
            '/uwb_anchor_distances',
            10
        )

        self.anchors = [
            [-3.0, -3.0, 1.0],
            [3.0, -3.0, 2.0],
            [-3.0, 0.0, 3.0],
            [3.0, 0.0, 1.5],
            [-3.0, 3.0, 2.5],
            [3.0, 3.0, 3.5]
        ]

        self.noise_mean = 0.0
        self.noise_std = 0.20

        self.timer = self.create_timer(
            0.1,
            self.publish_anchor_distances
        )

        self.get_logger().info("Anchor Distance Simulator Started")

    def publish_anchor_distances(self):

        distances = []

        for i in range(len(self.anchors)):
            for j in range(i + 1, len(self.anchors)):
                anchor_i = self.anchors[i]
                anchor_j = self.anchors[j]

                dx = anchor_i[0] - anchor_j[0]
                dy = anchor_i[1] - anchor_j[1]
                dz = anchor_i[2] - anchor_j[2]

                distance = math.sqrt(dx**2 + dy**2 + dz**2)
                distance += random.gauss(self.noise_mean, self.noise_std)

                distances.extend([
                    float(i),
                    float(j),
                    float(round(distance, 3))
                ])

        msg = Float64MultiArray()
        msg.data = distances

        self.publisher_.publish(msg)

        self.get_logger().info(
            "Published 15 anchor pair distances"
        )


def main(args=None):

    rclpy.init(args=args)

    node = AnchorDistanceSimulator()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
