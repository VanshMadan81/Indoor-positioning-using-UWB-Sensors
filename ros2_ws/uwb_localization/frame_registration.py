import numpy as np

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point
from std_msgs.msg import Float64MultiArray


class FrameRegistration(Node):

    def __init__(self):

        super().__init__('frame_registration')

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

        self.ls_world_pub = self.create_publisher(
            Point,
            '/ls_position_world',
            10
        )

        self.ekf_world_pub = self.create_publisher(
            Point,
            '/ekf_position_world',
            10
        )

        self.anchor_world_pub = self.create_publisher(
            Float64MultiArray,
            '/estimated_anchor_positions_world',
            10
        )

        self.world_anchors = np.array([
            [-3.0, -3.0, 1.0],
            [ 3.0, -3.0, 2.0],
            [-3.0,  0.0, 3.0],
            [ 3.0,  0.0, 1.5],
            [-3.0,  3.0, 2.5],
            [ 3.0,  3.0, 3.5]
        ])

        self.rotation_asl_to_world = None
        self.translation_asl_to_world = None

        self.transform_ready = False

        self.get_logger().info(
            "Frame Registration Started"
        )

    def anchor_callback(self, msg):

        if len(msg.data) != 18:
            self.get_logger().warn(
                f"Expected 18 values, received {len(msg.data)}"
            )
            return

        anchors_asl = np.array(
            msg.data
        ).reshape((6, 3))

        (
            self.rotation_asl_to_world,
            self.translation_asl_to_world
        ) = self.kabsch_alignment(
            self.world_anchors,
            anchors_asl
        )

        self.transform_ready = True

        anchors_world = (
            self.rotation_asl_to_world @ anchors_asl.T
        ).T + self.translation_asl_to_world

        out = Float64MultiArray()
        out.data = anchors_world.flatten().tolist()

        self.anchor_world_pub.publish(out)

        self.get_logger().info(
            "Published world anchors"
        )

    def ls_callback(self, msg):

        if not self.transform_ready:
            return

        p = self.transform_position(
            np.array([
                msg.x,
                msg.y,
                msg.z
            ])
        )

        self.ls_world_pub.publish(
            self.to_point(p)
        )

    def ekf_callback(self, msg):

        if not self.transform_ready:
            return

        p = self.transform_position(
            np.array([
                msg.x,
                msg.y,
                msg.z
            ])
        )

        self.ekf_world_pub.publish(
            self.to_point(p)
        )

    def transform_position(
        self,
        position_asl
    ):

        return (
            self.rotation_asl_to_world @ position_asl
            + self.translation_asl_to_world
        )

    def to_point(
        self,
        position
    ):

        msg = Point()

        msg.x = float(position[0])
        msg.y = float(position[1])
        msg.z = float(position[2])

        return msg

    def kabsch_alignment(
        self,
        target,
        source
    ):

        target_centroid = np.mean(
            target,
            axis=0
        )

        source_centroid = np.mean(
            source,
            axis=0
        )

        target_centered = (
            target -
            target_centroid
        )

        source_centered = (
            source -
            source_centroid
        )

        covariance = (
            source_centered.T
            @ target_centered
        )

        u, _, vt = np.linalg.svd(
            covariance
        )

        rotation = vt.T @ u.T

        translation = (
            target_centroid
            - rotation @ source_centroid
        )

        return rotation, translation


def main(args=None):

    rclpy.init(args=args)

    node = FrameRegistration()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()