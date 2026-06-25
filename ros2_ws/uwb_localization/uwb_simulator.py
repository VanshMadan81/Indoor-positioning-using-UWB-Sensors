from std_msgs.msg import Float64MultiArray
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry

import math
import random


class UWBSimulator(Node):

    def __init__(self):

        super().__init__('uwb_simulator')

        self.declare_parameter(
            'robot_odom_topic',
            '/odom'
        )

        robot_topic = (
            self.get_parameter('robot_odom_topic')
            .get_parameter_value()
            .string_value
        )

        self.subscription = self.create_subscription(
            Odometry,
            robot_topic,
            self.odom_callback,
            10
        )

        self.get_logger().info(
            f"Listening to robot odometry on {robot_topic}"
        )
        self.publisher_ = self.create_publisher(
        Float64MultiArray,
        '/uwb_distances',
        10
        )

        self.anchors = [
            [-3.0, -3.0, 1.0],      # A1
            [3.0, -3.0, 2.0],     # A2
            [-3.0, 0.0, 3.0],      # A3
            [3.0, 0.0, 1.5],     # A4
            [-3.0, 3.0, 2.5],     # A5
            [3.0, 3.0, 3.5]     # A6
        ]

        self.get_logger().info("UWB Simulator Started")

    def odom_callback(self, msg):

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z

        distances = []

        for anchor in self.anchors:

            dx = x - anchor[0]
            dy = y - anchor[1]
            dz = z - anchor[2]

            distance = math.sqrt(dx**2 + dy**2+ dz**2)

            noise = random.gauss(0, 0.20)

            distance = distance + noise

            distances.append(round(distance, 3))
        msg_out = Float64MultiArray()
        msg_out.data = distances

        self.publisher_.publish(msg_out)

        self.get_logger().info(
            f"Robot=({x:.2f},{y:.2f},{z:.2f})  Distances={distances}"
        )


def main(args=None):

    rclpy.init(args=args)

    node = UWBSimulator()
    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
