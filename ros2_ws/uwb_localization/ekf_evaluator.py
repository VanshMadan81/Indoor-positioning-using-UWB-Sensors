import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point

import math


class PositionEvaluator(Node):

    def __init__(self):

        super().__init__('position_evaluator')

        self.true_x = None
        self.true_y = None

        self.est_x = None
        self.est_y = None
        self.true_z = None

        self.est_z = None

        self.total_error = 0.0
        self.count = 0

        self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.create_subscription(
            Point,
            '/ekf_position_world',
            self.ekf_callback,
            10
        )

    def odom_callback(self, msg):

        self.true_x = msg.pose.pose.position.x
        self.true_y = msg.pose.pose.position.y
        self.true_z = msg.pose.pose.position.z

    def ekf_callback(self, msg):

        self.est_x = msg.x
        self.est_y = msg.y
        self.est_z = msg.z

        if self.true_x is None:
            return

        error = math.sqrt(
            (self.true_x - self.est_x)**2 +
            (self.true_y - self.est_y)**2 +
            (self.true_z - self.est_z)**2
        )
        self.total_error += error
        self.count += 1

        avg_error = self.total_error / self.count

        with open('/home/vansh/ros2_ws/ekf_errors_raw.txt', 'a') as f:
            f.write(f"{error}\n")

        self.get_logger().info(
            f"Error={error:.3f} m | "
            f"Average={avg_error:.3f} m | "
            f"Samples={self.count}"
        )


def main(args=None):

    rclpy.init(args=args)

    node = PositionEvaluator()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
