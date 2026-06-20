from __future__ import annotations

import csv
import math
from copy import deepcopy
from pathlib import Path
from typing import Any

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from px4_msgs.msg import VehicleOdometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


def _float(row: dict[str, str], key: str, default: float = 0.0) -> float:
    value = row.get(key, "")
    try:
        if value == "":
            return default
        return float(value)
    except (TypeError, ValueError):
        return default


def _first_existing(row: dict[str, str], keys: list[str], default: float = 0.0) -> tuple[str, float]:
    for key in keys:
        if key in row:
            return key, _float(row, key, default)
    return "none", default


class DatasetReplayNode(Node):
    """Injects a real degradation profile into live PX4 odometry.

    The node keeps Gazebo/PX4 as the live 3D source, then uses rows from a real
    CSV dataset to perturb the odometry sent to TwinGuard. This keeps the video
    tied to the simulator while validating the integrity layer against a real
    time-series degradation profile.
    """

    def __init__(self) -> None:
        super().__init__("twinguard_dataset_replay")
        self.dataset_csv = Path(self.declare_parameter("dataset_csv", "").value).expanduser()
        self.replay_rate_hz = float(self.declare_parameter("replay_rate_hz", 20.0).value)
        self.error_scale = float(self.declare_parameter("error_scale", 0.1).value)
        self.max_offset_m = float(self.declare_parameter("max_offset_m", 3.0).value)
        self.loop = bool(self.declare_parameter("loop", True).value)
        self.axis = str(self.declare_parameter("perturb_axis", "x").value).lower()

        self.rows = self._load_rows(self.dataset_csv)
        self.row_index = 0
        self.latest_odometry: VehicleOdometry | None = None

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.odom_sub = self.create_subscription(
            VehicleOdometry,
            "input_vehicle_odometry",
            self._handle_odometry,
            qos,
        )
        self.odom_pub = self.create_publisher(
            VehicleOdometry,
            "output_vehicle_odometry",
            qos,
        )
        self.diag_pub = self.create_publisher(
            DiagnosticArray,
            "dataset_replay_diagnostics",
            10,
        )

        period_s = 1.0 / max(self.replay_rate_hz, 1e-3)
        self.timer = self.create_timer(period_s, self._tick)
        self.get_logger().info(
            f"Loaded {len(self.rows)} dataset rows from {self.dataset_csv}"
        )

    def _load_rows(self, path: Path) -> list[dict[str, str]]:
        if not path:
            raise RuntimeError("dataset_csv parameter is required")
        if not path.exists():
            raise RuntimeError(f"dataset_csv does not exist: {path}")
        with path.open("r", encoding="utf-8", newline="") as f:
            rows = list(csv.DictReader(f))
        if not rows:
            raise RuntimeError(f"dataset_csv contains no rows: {path}")
        return rows

    def _handle_odometry(self, msg: VehicleOdometry) -> None:
        self.latest_odometry = msg

    def _tick(self) -> None:
        if self.latest_odometry is None:
            self._publish_diag("waiting_for_px4_odometry", {}, DiagnosticStatus.STALE)
            return

        row = self.rows[self.row_index]
        source_col, source_error_m = _first_existing(
            row,
            [
                "measurement_error_m",
                "est_err_m",
                "dt_err_m",
                "sensor_residual_m",
                "innovation_norm_m",
                "nis",
            ],
            0.0,
        )
        _, quality = _first_existing(row, ["quality", "true_quality"], 1.0)
        _, is_outlier = _first_existing(row, ["is_outlier", "ids_anomaly_flag"], 0.0)

        applied_offset = self._offset_from_dataset(
            source_error_m=source_error_m,
            quality=quality,
            is_outlier=is_outlier,
        )

        out = deepcopy(self.latest_odometry)
        axis_idx = {"x": 0, "y": 1, "z": 2}.get(self.axis, 0)
        out.position[axis_idx] = float(out.position[axis_idx] + applied_offset)
        self.odom_pub.publish(out)

        self._publish_diag(
            "dataset_replay_active",
            {
                "row_index": self.row_index,
                "source_column": source_col,
                "dataset_error_m": source_error_m,
                "quality": quality,
                "is_outlier": is_outlier,
                "applied_offset_m": applied_offset,
            },
            DiagnosticStatus.OK,
        )

        self.row_index += 1
        if self.row_index >= len(self.rows):
            self.row_index = 0 if self.loop else len(self.rows) - 1

    def _offset_from_dataset(self, source_error_m: float, quality: float, is_outlier: float) -> float:
        quality = min(max(quality, 0.0), 1.0)
        quality_gain = 1.0 + (1.0 - quality)
        outlier_gain = 2.0 if is_outlier >= 0.5 else 1.0
        raw_offset = abs(source_error_m) * self.error_scale * quality_gain * outlier_gain
        if not math.isfinite(raw_offset):
            raw_offset = 0.0
        return min(raw_offset, self.max_offset_m)

    def _publish_diag(self, message: str, values: dict[str, Any], level: int) -> None:
        status = DiagnosticStatus()
        status.name = "twinguard/dataset_replay"
        status.hardware_id = "real_dataset_profile"
        status.level = level
        status.message = message
        status.values = [KeyValue(key=k, value=str(v)) for k, v in values.items()]

        diag = DiagnosticArray()
        diag.header.stamp = self.get_clock().now().to_msg()
        diag.status.append(status)
        self.diag_pub.publish(diag)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = DatasetReplayNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
