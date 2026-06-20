from __future__ import annotations

import math
import time
from dataclasses import dataclass

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PointStamped
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand
from rclpy.node import Node


@dataclass
class MissionState:
    tick_count: int = 0
    offboard_requested: bool = False
    arm_requested: bool = False
    trust: float = 1.0
    authority_scale: float = 1.0


class OffboardMissionController(Node):
    """PX4 offboard mission controller with TwinGuard trust-aware setpoints.

    The node intentionally keeps mission generation separate from integrity
    scoring. PX4/Gazebo provides the vehicle dynamics, TwinGuard publishes
    trust, and this controller converts trust into conservative setpoint
    authority for recording repeatable UAV mission demos.
    """

    def __init__(self) -> None:
        super().__init__("twinguard_offboard_mission_controller")

        self.declare_parameter("drone_id", 0)
        self.declare_parameter("px4_prefix", "")
        self.declare_parameter("trust_topic", "trust_state")
        self.declare_parameter("takeoff_altitude_m", 2.0)
        self.declare_parameter("mission_radius_m", 3.0)
        self.declare_parameter("mission_period_s", 18.0)
        self.declare_parameter("center_x_m", 0.0)
        self.declare_parameter("center_y_m", 0.0)
        self.declare_parameter("min_authority_scale", 0.25)
        self.declare_parameter("setpoint_rate_hz", 20.0)
        self.declare_parameter("auto_arm", True)
        self.declare_parameter("force_arm", False)
        self.declare_parameter("mission_mode", "circle")

        self.drone_id = int(self.get_parameter("drone_id").value)
        self.px4_prefix = self._normalize_prefix(str(self.get_parameter("px4_prefix").value))
        self.takeoff_altitude_m = float(self.get_parameter("takeoff_altitude_m").value)
        self.mission_radius_m = float(self.get_parameter("mission_radius_m").value)
        self.mission_period_s = max(float(self.get_parameter("mission_period_s").value), 1.0)
        self.center_x_m = float(self.get_parameter("center_x_m").value)
        self.center_y_m = float(self.get_parameter("center_y_m").value)
        self.min_authority_scale = float(self.get_parameter("min_authority_scale").value)
        self.auto_arm = bool(self.get_parameter("auto_arm").value)
        self.force_arm = bool(self.get_parameter("force_arm").value)
        self.mission_mode = str(self.get_parameter("mission_mode").value).lower()

        setpoint_rate_hz = max(float(self.get_parameter("setpoint_rate_hz").value), 2.0)
        trust_topic = str(self.get_parameter("trust_topic").value)

        self.state = MissionState(authority_scale=1.0)
        self.start_time = time.monotonic()

        self.offboard_pub = self.create_publisher(
            OffboardControlMode, self._topic("fmu/in/offboard_control_mode"), 10
        )
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint, self._topic("fmu/in/trajectory_setpoint"), 10
        )
        self.command_pub = self.create_publisher(
            VehicleCommand, self._topic("fmu/in/vehicle_command"), 10
        )
        self.diag_pub = self.create_publisher(DiagnosticArray, "mission_diagnostics", 10)
        self.create_subscription(PointStamped, trust_topic, self._on_trust_state, 10)

        self.timer = self.create_timer(1.0 / setpoint_rate_hz, self._tick)
        self.get_logger().info(
            "TwinGuard offboard mission controller ready: "
            f"drone_id={self.drone_id}, px4_prefix='{self.px4_prefix or '/'}', "
            f"mode={self.mission_mode}, radius={self.mission_radius_m:.2f} m"
        )

    @staticmethod
    def _normalize_prefix(prefix: str) -> str:
        prefix = prefix.strip("/")
        return f"/{prefix}" if prefix else ""

    def _topic(self, relative_name: str) -> str:
        return f"{self.px4_prefix}/{relative_name.lstrip('/')}"

    def _timestamp_us(self) -> int:
        return int(self.get_clock().now().nanoseconds / 1000)

    def _on_trust_state(self, msg: PointStamped) -> None:
        trust = float(max(0.0, min(1.0, msg.point.x)))
        authority = float(max(self.min_authority_scale, min(1.0, msg.point.z)))
        self.state.trust = trust
        self.state.authority_scale = authority

    def _tick(self) -> None:
        self.state.tick_count += 1
        timestamp = self._timestamp_us()

        self._publish_offboard_mode(timestamp)
        self._publish_setpoint(timestamp)

        # PX4 requires a short stream of setpoints before accepting OFFBOARD.
        if self.auto_arm and self.state.tick_count in (20, 60):
            self._request_offboard_mode(timestamp)
            self._arm(timestamp)

        self._publish_diagnostics(timestamp)

    def _publish_offboard_mode(self, timestamp: int) -> None:
        msg = OffboardControlMode()
        msg.timestamp = timestamp
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        self.offboard_pub.publish(msg)

    def _publish_setpoint(self, timestamp: int) -> None:
        elapsed_s = time.monotonic() - self.start_time
        authority = self.state.authority_scale
        radius = self.mission_radius_m * authority

        if self.mission_mode == "hold":
            x = self.center_x_m
            y = self.center_y_m
            yaw = 0.0
        else:
            phase = 2.0 * math.pi * elapsed_s / self.mission_period_s
            x = self.center_x_m + radius * math.cos(phase)
            y = self.center_y_m + radius * math.sin(phase)
            yaw = math.atan2(y - self.center_y_m, x - self.center_x_m) + math.pi / 2.0

        msg = TrajectorySetpoint()
        msg.timestamp = timestamp
        msg.position = [float(x), float(y), float(-self.takeoff_altitude_m)]
        msg.velocity = [math.nan, math.nan, math.nan]
        msg.acceleration = [math.nan, math.nan, math.nan]
        msg.jerk = [math.nan, math.nan, math.nan]
        msg.yaw = float(yaw)
        msg.yawspeed = math.nan
        self.setpoint_pub.publish(msg)

    def _request_offboard_mode(self, timestamp: int) -> None:
        self._publish_vehicle_command(
            timestamp,
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            param1=1.0,
            param2=6.0,
        )
        self.state.offboard_requested = True

    def _arm(self, timestamp: int) -> None:
        self._publish_vehicle_command(
            timestamp,
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=1.0,
            param2=21196.0 if self.force_arm else 0.0,
        )
        self.state.arm_requested = True

    def _publish_vehicle_command(
        self,
        timestamp: int,
        command: int,
        *,
        param1: float = 0.0,
        param2: float = 0.0,
        param3: float = 0.0,
        param4: float = 0.0,
        param5: float = 0.0,
        param6: float = 0.0,
        param7: float = 0.0,
    ) -> None:
        msg = VehicleCommand()
        msg.timestamp = timestamp
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.param3 = float(param3)
        msg.param4 = float(param4)
        msg.param5 = float(param5)
        msg.param6 = float(param6)
        msg.param7 = float(param7)
        msg.command = int(command)
        msg.target_system = self.drone_id + 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.command_pub.publish(msg)

    def _publish_diagnostics(self, timestamp: int) -> None:
        diag = DiagnosticArray()
        diag.header.stamp = self.get_clock().now().to_msg()

        status = DiagnosticStatus()
        status.name = f"twinguard/offboard_mission_drone_{self.drone_id}"
        status.hardware_id = f"uav_{self.drone_id}"
        status.level = DiagnosticStatus.OK
        status.message = "offboard_mission_active"
        status.values = [
            KeyValue(key="timestamp_us", value=str(timestamp)),
            KeyValue(key="mission_mode", value=self.mission_mode),
            KeyValue(key="trust", value=f"{self.state.trust:.6f}"),
            KeyValue(key="authority_scale", value=f"{self.state.authority_scale:.6f}"),
            KeyValue(key="offboard_requested", value=str(self.state.offboard_requested)),
            KeyValue(key="arm_requested", value=str(self.state.arm_requested)),
            KeyValue(key="force_arm", value=str(self.force_arm)),
        ]
        diag.status.append(status)
        self.diag_pub.publish(diag)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OffboardMissionController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
