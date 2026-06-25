import numpy as np
import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
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
            [3.0, -3.0, 2.0],
            [-3.0, 0.0, 3.0],
            [3.0, 0.0, 1.5],
            [-3.0, 3.0, 2.5],
            [3.0, 3.0, 3.5]
        ])

        self.estimated_anchors = None
        self.rotation_asl_to_world = None
        self.translation_asl_to_world = None
        self.transform_ready = False

        self.get_logger().info("Frame Registration Started")

    def anchor_callback(self, msg):

        if len(msg.data) != 18:
            self.get_logger().warn(
                f"Expected 18 anchor values, received {len(msg.data)}"
            )
            return

        estimated_anchors = np.array(msg.data).reshape((6, 3))

        if (
            self.estimated_anchors is not None and
            np.allclose(estimated_anchors, self.estimated_anchors)
        ):
            return

        self.estimated_anchors = estimated_anchors

        self.rotation_asl_to_world, self.translation_asl_to_world = (
            self.kabsch_alignment(
                self.world_anchors,
                self.estimated_anchors
            )
        )

        self.transform_ready = True

        self.get_logger().info("Updated ASL to world transform")
        self.get_logger().info(
            f"det(R) = {np.linalg.det(self.rotation_asl_to_world):.6f}"
        )

        self.publish_world_anchors()

    def ls_callback(self, msg):

        if not self.transform_ready:
            return

        position_world = self.transform_position(
            np.array([
                msg.x,
                msg.y,
                msg.z
            ])
        )

        self.ls_world_pub.publish(
            self.to_point(position_world)
        )
        self.get_logger().info("Publishing world positions")

    def ekf_callback(self, msg):

        if not self.transform_ready:
            return

        position_world = self.transform_position(
            np.array([
                msg.x,
                msg.y,
                msg.z
            ])
        )

        self.ekf_world_pub.publish(
            self.to_point(position_world)
        )
        self.get_logger().info("Publishing world positions")

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

    def transform_position(self, position_asl):

        return (
            self.rotation_asl_to_world @ position_asl +
            self.translation_asl_to_world
        )

    def to_point(self, position):

        point = Point()
        point.x = float(position[0])
        point.y = float(position[1])
        point.z = float(position[2])

        return point

    def publish_world_anchors(self):

        if self.estimated_anchors is None or not self.transform_ready:
            return

        anchors_world = np.array([
            self.transform_position(anchor)
            for anchor in self.estimated_anchors
        ])

        msg = Float64MultiArray()
        msg.data = anchors_world.flatten().tolist()

        self.anchor_world_pub.publish(msg)
        self.get_logger().info("Publishing world anchors")


def main(args=None):

    rclpy.init(args=args)

    node = FrameRegistration()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
