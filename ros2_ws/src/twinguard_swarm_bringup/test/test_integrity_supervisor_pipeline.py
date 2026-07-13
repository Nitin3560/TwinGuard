import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from px4_msgs.msg import VehicleOdometry
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy


@pytest.mark.launch_test
def generate_test_description():
    integrity_node = launch_ros.actions.Node(
        package="twinguard_swarm_integrity_cpp",
        executable="integrity_node_cpp",
        name="integrity_node_cpp",
        parameters=[{"stale_timeout_ms": 1000, "prediction_dt": 0.1}],
    )
    supervisor_node = launch_ros.actions.Node(
        package="twinguard_swarm_integrity_cpp",
        executable="formation_supervisor_node",
        name="formation_supervisor_node",
        parameters=[
            {
                "auto_arm": False,
                "force_arm": False,
                "setpoint_rate_hz": 20.0,
                "stale_timeout_ms": 1000,
                "mission_mode": "hold",
            }
        ],
    )
    return launch.LaunchDescription(
        [
            integrity_node,
            supervisor_node,
            launch_testing.actions.ReadyToTest(),
        ]
    ), {
        "integrity_node": integrity_node,
        "supervisor_node": supervisor_node,
    }


class TestIntegritySupervisorPipeline(unittest.TestCase):

    def setUp(self):
        rclpy.init()
        self.node = rclpy.create_node("test_integrity_supervisor_pipeline")
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.odom_pub = self.node.create_publisher(
            VehicleOdometry, "fmu/out/vehicle_odometry", qos
        )
        self.integrity_diags = []
        self.supervisor_diags = []
        self.node.create_subscription(
            DiagnosticArray,
            "integrity_diagnostics",
            lambda msg: self.integrity_diags.append(msg),
            10,
        )
        self.node.create_subscription(
            DiagnosticArray,
            "supervisor_diagnostics",
            lambda msg: self.supervisor_diags.append(msg),
            10,
        )

    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown()

    def _publish_odometry(self, position, velocity=None):
        msg = VehicleOdometry()
        msg.position = [float(position[0]), float(position[1]), float(position[2])]
        vel = velocity if velocity is not None else [0.0, 0.0, 0.0]
        msg.velocity = [float(vel[0]), float(vel[1]), float(vel[2])]
        self.odom_pub.publish(msg)

    def _spin_until(self, predicate, timeout_s=8.0):
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if predicate():
                return True
            time.sleep(0.05)
        return False

    def test_large_odometry_jump_reaches_supervisor(self):
        for _ in range(5):
            self._publish_odometry([0.0, 0.0, 0.0])
            rclpy.spin_once(self.node, timeout_sec=0.1)
            time.sleep(0.05)

        def pipeline_degraded():
            self._publish_odometry([50.0, 0.0, 0.0])
            integrity_warn = any(
                diag.status and diag.status[0].level >= DiagnosticStatus.WARN
                for diag in self.integrity_diags
            )
            supervisor_reacted = any(
                diag.status
                and diag.status[0].message in ("recovering", "degraded_hold")
                for diag in self.supervisor_diags
            )
            return integrity_warn and supervisor_reacted

        self.assertTrue(
            self._spin_until(pipeline_degraded),
            "integrity warning did not propagate into supervisor behavior",
        )
