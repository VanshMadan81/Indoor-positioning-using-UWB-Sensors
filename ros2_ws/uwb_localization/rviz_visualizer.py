import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray

import numpy as np


MAX_PATH_POINTS = 1000


class RVizVisualizer(Node):

    def __init__(self):

        super().__init__('rviz_visualizer')

        self.create_subscription(
            Point,
            '/ls_position_world',
            self.ls_callback,
            10
        )

        self.create_subscription(
            Point,
            '/ekf_position_world',
            self.ekf_callback,
            10
        )

        self.create_subscription(
            Odometry,
            '/odom',
            self.truth_callback,
            10
        )

        self.create_subscription(
            Float64MultiArray,
            '/estimated_anchor_positions_world',
            self.anchor_callback,
            10
        )

        self.truth_pub = self.create_publisher(
            Marker,
            '/truth_marker',
            10
        )

        self.ls_pub = self.create_publisher(
            Marker,
            '/ls_marker',
            10
        )

        self.ekf_pub = self.create_publisher(
            Marker,
            '/ekf_marker',
            10
        )

        self.truth_path_pub = self.create_publisher(
            Path,
            '/truth_path',
            10
        )

        self.ls_path_pub = self.create_publisher(
            Path,
            '/ls_path',
            10
        )

        self.ekf_path_pub = self.create_publisher(
            Path,
            '/ekf_path',
            10
        )

        self.truth_path = Path()
        self.ls_path = Path()
        self.ekf_path = Path()

        self.truth_path.header.frame_id = "odom"
        self.ls_path.header.frame_id = "odom"
        self.ekf_path.header.frame_id = "odom"

        self.anchor_pub = self.create_publisher(
            Marker,
            '/anchor_markers',
            10
        )

        self.timer = self.create_timer(
            1.0,
            self.publish_anchors
        )

        self.anchors = np.array([
            [-3.0, -3.0, 1.0],
            [ 3.0, -3.0, 2.0],
            [-3.0,  0.0, 3.0],
            [ 3.0,  0.0, 1.5],
            [-3.0,  3.0, 2.5],
            [ 3.0,  3.0, 3.5]
        ])

    def anchor_callback(self, msg):

        if len(msg.data) != 18:
            return

        self.anchors = np.array(msg.data).reshape((6, 3))

    def append_pose(self, path, pose):

        path.poses.append(pose)

        #if len(path.poses) > MAX_PATH_POINTS:
           # path.poses.pop(0)

    def ls_callback(self, msg):

        marker = Marker()

        marker.header.frame_id = "odom"
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = "ls"
        marker.id = 0

        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = msg.x
        marker.pose.position.y = msg.y
        marker.pose.position.z = msg.z

        marker.scale.x = 0.20
        marker.scale.y = 0.20
        marker.scale.z = 0.20

        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0

        self.ls_pub.publish(marker)

        pose = PoseStamped()

        pose.header.frame_id = "odom"
        pose.header.stamp = marker.header.stamp

        pose.pose.position.x = msg.x
        pose.pose.position.y = msg.y
        pose.pose.position.z = msg.z

        self.append_pose(self.ls_path, pose)

        self.ls_path.header.stamp = marker.header.stamp
        self.ls_path_pub.publish(self.ls_path)

    def ekf_callback(self, msg):

        marker = Marker()

        marker.header.frame_id = "odom"
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = "ekf"
        marker.id = 1

        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = msg.x
        marker.pose.position.y = msg.y
        marker.pose.position.z = msg.z

        marker.scale.x = 0.20
        marker.scale.y = 0.20
        marker.scale.z = 0.20

        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        self.ekf_pub.publish(marker)

        pose = PoseStamped()

        pose.header.frame_id = "odom"
        pose.header.stamp = marker.header.stamp

        pose.pose.position.x = msg.x
        pose.pose.position.y = msg.y
        pose.pose.position.z = msg.z

        self.append_pose(self.ekf_path, pose)

        self.ekf_path.header.stamp = marker.header.stamp
        self.ekf_path_pub.publish(self.ekf_path)

    def truth_callback(self, msg):

        marker = Marker()

        marker.header.frame_id = "odom"
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = "truth"
        marker.id = 2

        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = msg.pose.pose.position.x
        marker.pose.position.y = msg.pose.pose.position.y
        marker.pose.position.z = msg.pose.pose.position.z

        marker.scale.x = 0.30
        marker.scale.y = 0.30
        marker.scale.z = 0.30

        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        self.truth_pub.publish(marker)

        pose = PoseStamped()

        pose.header.frame_id = "odom"
        pose.header.stamp = marker.header.stamp

        pose.pose.position.x = msg.pose.pose.position.x
        pose.pose.position.y = msg.pose.pose.position.y
        pose.pose.position.z = msg.pose.pose.position.z

        self.append_pose(self.truth_path, pose)

        self.truth_path.header.stamp = marker.header.stamp
        self.truth_path_pub.publish(self.truth_path)

    def publish_anchors(self):

        current_time = self.get_clock().now().to_msg()

        for i, anchor in enumerate(self.anchors):

            marker = Marker()

            marker.header.frame_id = "odom"
            marker.header.stamp = current_time

            marker.ns = "anchors"
            marker.id = i

            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            marker.pose.position.x = float(anchor[0])
            marker.pose.position.y = float(anchor[1])
            marker.pose.position.z = float(anchor[2])

            marker.scale.x = 0.40
            marker.scale.y = 0.40
            marker.scale.z = 0.40

            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0

            self.anchor_pub.publish(marker)


def main(args=None):

    rclpy.init(args=args)

    node = RVizVisualizer()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()