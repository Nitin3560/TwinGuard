#!/usr/bin/env python3
from geometry_msgs.msg import Point, PointStamped, TransformStamped
from px4_msgs.msg import VehicleOdometry
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import ColorRGBA
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker


class TrustVisualizerNode(Node):
    def __init__(self):
        super().__init__("trust_visualizer_node")
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.tf_broadcaster = TransformBroadcaster(self)
        self.marker_pub = self.create_publisher(Marker, "/twinguard/drone_marker", 10)
        self.trail_pub = self.create_publisher(Marker, "/twinguard/trust_trail", 10)
        self.trust = 1.0
        self.position = [0.0, 0.0, -2.0]
        self.trail_points = []

        self.create_subscription(VehicleOdometry, "/fmu/out/vehicle_odometry", self._on_odometry, qos)
        self.create_subscription(PointStamped, "/twinguard/trust_state", self._on_trust, 10)
        self.create_timer(0.1, self._publish_visuals)

    def _on_odometry(self, msg):
        self.position = [msg.position[0], msg.position[1], msg.position[2]]

    def _on_trust(self, msg):
        self.trust = msg.point.x

    def _publish_visuals(self):
        now = self.get_clock().now().to_msg()
        x, y, z_ned = self.position
        z_up = -z_ned

        transform = TransformStamped()
        transform.header.stamp = now
        transform.header.frame_id = "map"
        transform.child_frame_id = "base_link"
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.translation.z = z_up
        transform.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(transform)

        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = now
        marker.ns = "twinguard"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z_up
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.5

        trust = max(0.0, min(1.0, self.trust))
        if trust > 0.5:
            marker.color.r = (1.0 - trust) * 2.0
            marker.color.g = 1.0
        else:
            marker.color.r = 1.0
            marker.color.g = trust * 2.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        self.marker_pub.publish(marker)

        self.trail_points.append((x, y, z_up, marker.color.r, marker.color.g))
        if len(self.trail_points) > 300:
            self.trail_points.pop(0)

        trail = Marker()
        trail.header.frame_id = "map"
        trail.header.stamp = now
        trail.ns = "twinguard"
        trail.id = 1
        trail.type = Marker.LINE_STRIP
        trail.action = Marker.ADD
        trail.scale.x = 0.08
        trail.pose.orientation.w = 1.0
        for px, py, pz, red, green in self.trail_points:
            trail.points.append(Point(x=px, y=py, z=pz))
            trail.colors.append(ColorRGBA(r=red, g=green, b=0.0, a=1.0))
        self.trail_pub.publish(trail)


def main():
    rclpy.init()
    node = TrustVisualizerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
