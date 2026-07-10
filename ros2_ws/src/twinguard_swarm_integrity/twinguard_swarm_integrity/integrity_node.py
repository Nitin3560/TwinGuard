from __future__ import annotations

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node
from std_msgs.msg import Float32


class IntegrityNode(Node):
    """Placeholder ROS2 binding for TwinGuard trust diagnostics."""

    def __init__(self) -> None:
        super().__init__("twinguard_integrity_node")
        self.declare_parameter("drone_id", 0)
        self.trust_pub = self.create_publisher(Float32, "trust", 10)
        self.diag_pub = self.create_publisher(DiagnosticArray, "diagnostics", 10)
        self.timer = self.create_timer(0.1, self._tick)

    def _tick(self) -> None:
        msg = Float32()
        msg.data = 1.0
        self.trust_pub.publish(msg)

        diag = DiagnosticArray()
        status = DiagnosticStatus()
        status.name = "twinguard/integrity"
        status.level = DiagnosticStatus.OK
        status.message = "waiting_for_px4_binding"
        status.values = [KeyValue(key="trust", value="1.0")]
        diag.status.append(status)
        self.diag_pub.publish(diag)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = IntegrityNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
