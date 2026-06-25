import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Point

import numpy as np


class EKFLocalizer(Node):

    def __init__(self):

        super().__init__('ekf_localizer')

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

        self.publisher_ = self.create_publisher(
            Point,
            '/ekf_position',
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

        # State = [x,y,z,vx,vy,vz]
        self.x = np.array([
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0
        ])

        self.P = np.eye(6)

        self.Q = np.diag([
            0.01,
            0.01,
            0.01,
            0.1,
            0.1,
            0.1
        ])

        self.R = np.eye(6) * 0.20**2

        self.dt = 0.1

        self.get_logger().info(
            "EKF Localizer Started"
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

    def distance_callback(self, msg):

        if not self.asl_anchors_ready:
            return

        z = np.array(msg.data)

        self.predict()

        self.update(z)

        point = Point()

        point.x = float(self.x[0])
        point.y = float(self.x[1])
        point.z = float(self.x[2])

        self.publisher_.publish(point)

        self.get_logger().info(
            f"EKF Position = ({self.x[0]:.2f}, {self.x[1]:.2f}, {self.x[2]:.2f})"
        )

    def predict(self):

        F = np.array([
            [1,0,0,self.dt,0,0],
            [0,1,0,0,self.dt,0],
            [0,0,1,0,0,self.dt],
            [0,0,0,1,0,0],
            [0,0,0,0,1,0],
            [0,0,0,0,0,1]
        ])

        self.x = F @ self.x

        self.P = F @ self.P @ F.T + self.Q

    def update(self, z):

        x_pos = self.x[0]
        y_pos = self.x[1]
        z_pos = self.x[2]

        h = []
        H = []

        for anchor in self.anchors:

            dx = x_pos - anchor[0]
            dy = y_pos - anchor[1]
            dz = z_pos - anchor[2]

            d = np.sqrt(dx**2 + dy**2 + dz**2)

            if d < 0.001:
                d = 0.001

            h.append(d)

            H.append([
                dx/d,
                dy/d,
                dz/d,
                0,
                0,
                0
            ])

        h = np.array(h)
        H = np.array(H)

        y = z - h

        S = H @ self.P @ H.T + self.R

        K = self.P @ H.T @ np.linalg.inv(S)

        self.x = self.x + K @ y

        I = np.eye(6)

        self.P = (I - K @ H) @ self.P


def main(args=None):

    rclpy.init(args=args)

    node = EKFLocalizer()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
