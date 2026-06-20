from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="twinguard_swarm_integrity_cpp",
                executable="integrity_node_cpp",
                name="integrity_drone_0_cpp",
                namespace="drone_0/twinguard",
                output="screen",
                parameters=[
                    {
                        "drone_id": 0,
                        "stale_timeout_ms": 500,
                        "prediction_dt": 0.1,
                    }
                ],
                remappings=[
                    ("fmu/out/vehicle_odometry", "/drone_0/fmu/out/vehicle_odometry"),
                ],
            ),
            Node(
                package="twinguard_swarm_integrity_cpp",
                executable="formation_supervisor_node",
                name="formation_supervisor_drone_0",
                namespace="drone_0/twinguard",
                output="screen",
                parameters=[
                    {
                        "drone_id": 0,
                        "target_system": 1,
                        "nominal_x_m": 0.0,
                        "nominal_y_m": 0.0,
                        "nominal_z_m": -2.0,
                        "nominal_velocity_limit_mps": 3.0,
                        "degraded_threshold": 0.5,
                        "stale_timeout_ms": 500,
                        "setpoint_rate_hz": 10.0,
                        "auto_arm": False,
                        "force_arm": False,
                    }
                ],
                remappings=[
                    ("fmu/out/vehicle_odometry", "/drone_0/fmu/out/vehicle_odometry"),
                    ("fmu/in/offboard_control_mode", "/drone_0/fmu/in/offboard_control_mode"),
                    ("fmu/in/trajectory_setpoint", "/drone_0/fmu/in/trajectory_setpoint"),
                    ("fmu/in/vehicle_command", "/drone_0/fmu/in/vehicle_command"),
                ],
            ),
        ]
    )
