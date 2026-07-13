import time
import unittest

import launch
import launch_ros.actions
import launch_testing.actions
import pytest
import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PointStamped
from px4_msgs.msg import TrajectorySetpoint, VehicleOdometry
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy


TEST_ALPHA = 1.2
TEST_BETA = 0.6
TEST_MIN_AUTHORITY = 0.15
STALE_TIMEOUT_MS = 2000


@pytest.mark.launch_test
def generate_test_description():
    integrity_node = launch_ros.actions.Node(
        package="twinguard_swarm_integrity_cpp",
        executable="integrity_node_cpp",
        name="integrity_node_cpp",
        parameters=[
            {
                "drone_id": 0,
                "stale_timeout_ms": STALE_TIMEOUT_MS,
                "prediction_dt": 0.1,
                "alpha": TEST_ALPHA,
                "beta": TEST_BETA,
                "min_authority": TEST_MIN_AUTHORITY,
            }
        ],
        output="screen",
    )
    supervisor_node = launch_ros.actions.Node(
        package="twinguard_swarm_integrity_cpp",
        executable="formation_supervisor_node",
        name="formation_supervisor_node",
        parameters=[
            {
                "drone_id": 0,
                "target_system": 1,
                "nominal_x_m": 0.0,
                "nominal_y_m": 0.0,
                "nominal_z_m": -2.0,
                "nominal_velocity_limit_mps": 3.0,
                "degraded_threshold": 0.5,
                "min_authority_scale": TEST_MIN_AUTHORITY,
                "stale_timeout_ms": STALE_TIMEOUT_MS,
                "setpoint_rate_hz": 10.0,
                "auto_arm": False,
                "force_arm": False,
                "mission_mode": "hold",
                "static_obstacle_enabled": False,
            }
        ],
        output="screen",
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


def _diag_values(msg: DiagnosticArray) -> dict:
    if not msg.status:
        return {}
    return {kv.key: kv.value for kv in msg.status[0].values}


class TestIntegritySupervisorPipeline(unittest.TestCase):

    def setUp(self):
        rclpy.init()
        self.node = rclpy.create_node("test_integrity_supervisor_pipeline_client")
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.odom_pub = self.node.create_publisher(
            VehicleOdometry, "fmu/out/vehicle_odometry", qos
        )
        self.latest_trust = None
        self.latest_supervisor_diag = None
        self.latest_setpoint = None
        self.node.create_subscription(
            PointStamped,
            "trust_state",
            lambda msg: setattr(self, "latest_trust", msg),
            10,
        )
        self.node.create_subscription(
            DiagnosticArray,
            "supervisor_diagnostics",
            lambda msg: setattr(self, "latest_supervisor_diag", msg),
            10,
        )
        self.node.create_subscription(
            TrajectorySetpoint,
            "fmu/in/trajectory_setpoint",
            lambda msg: setattr(self, "latest_setpoint", msg),
            10,
        )

    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown()

    def _publish_position(self, position, seconds):
        deadline = time.time() + seconds
        msg = VehicleOdometry()
        msg.position = [float(p) for p in position]
        msg.velocity = [0.0, 0.0, 0.0]
        while time.time() < deadline:
            self.odom_pub.publish(msg)
            rclpy.spin_once(self.node, timeout_sec=0.05)
            time.sleep(0.05)

    def _spin_for(self, seconds):
        deadline = time.time() + seconds
        while time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            time.sleep(0.02)

    def _wait_for_supervisor_mode(self, expected_modes, timeout_sec):
        deadline = time.time() + timeout_sec
        last_seen = None
        while time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if self.latest_supervisor_diag is not None:
                values = _diag_values(self.latest_supervisor_diag)
                last_seen = self.latest_supervisor_diag.status[0].message
                if last_seen in expected_modes:
                    return last_seen, values
            time.sleep(0.02)
        self.fail(
            f"Timed out after {timeout_sec}s waiting for supervisor mode in "
            f"{expected_modes}. Last observed mode: {last_seen!r}"
        )

    def test_trust_gated_pipeline_reacts_and_recovers(self):
        self._publish_position((0.0, 0.0, 0.0), seconds=1.0)
        mode, values = self._wait_for_supervisor_mode({"nominal"}, timeout_sec=3.0)
        self.assertEqual(mode, "nominal")
        self.assertEqual(values.get("hold"), "false")
        nominal_authority = float(values["authority_scale"])
        self.assertGreater(nominal_authority, 0.9)
        self.assertIsNotNone(self.latest_trust)
        self.assertLess(self.latest_trust.point.x, 1.01)
        self.assertGreater(self.latest_trust.point.x, 0.9)

        self._publish_position((10.0, 0.0, 0.0), seconds=0.5)
        mode, values = self._wait_for_supervisor_mode({"degraded_hold"}, timeout_sec=5.0)
        self.assertEqual(mode, "degraded_hold")
        degraded_authority = float(values["authority_scale"])
        self.assertLess(degraded_authority, nominal_authority)
        self.assertEqual(values.get("hold"), "true")
        self.assertEqual(float(values["velocity_limit_mps"]), 0.0)
        self.assertIsNotNone(self.latest_trust)
        self.assertLess(self.latest_trust.point.x, 0.35)

        self._spin_for(0.3)
        self.assertIsNotNone(self.latest_setpoint)
        frozen_position = list(self.latest_setpoint.position)
        self._spin_for(0.3)
        self.assertEqual(list(self.latest_setpoint.position), frozen_position)

        self._publish_position((0.0, 0.0, 0.0), seconds=1.0)
        mode, values = self._wait_for_supervisor_mode({"nominal"}, timeout_sec=6.0)
        self.assertEqual(mode, "nominal")
        self.assertEqual(values.get("hold"), "false")

        self._publish_position((0.0, 0.0, 0.0), seconds=1.5)
        rclpy.spin_once(self.node, timeout_sec=0.1)
        values = _diag_values(self.latest_supervisor_diag)
        recovered_authority = float(values["authority_scale"])
        self.assertGreater(recovered_authority, degraded_authority)
        self.assertGreater(recovered_authority, 0.9)
        self.assertIsNotNone(self.latest_trust)
        self.assertGreater(self.latest_trust.point.x, 0.9)
