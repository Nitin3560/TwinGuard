from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="twinguard_dataset_replay",
            executable="dataset_replay_node",
            name="dataset_replay",
            namespace="twinguard",
            output="screen",
            parameters=[{
                "dataset_csv": "/home/nitin/twinguard_ws/src/TwinGuard/demo/demo_profile.csv",
                "replay_rate_hz": 20.0,
                "error_scale": 1.0,
                "max_offset_m": 10.0,
                "loop": True,
                "perturb_axis": "x",
            }],
            remappings=[
                ("input_vehicle_odometry", "/fmu/out/vehicle_odometry"),
                ("output_vehicle_odometry", "/twinguard/replay/vehicle_odometry"),
            ],
        ),
        Node(
            package="twinguard_swarm_integrity_cpp",
            executable="integrity_node_cpp",
            name="integrity_node_cpp",
            namespace="twinguard",
            output="screen",
            parameters=[{
                "drone_id": 0,
                "stale_timeout_ms": 2000,
                "prediction_dt": 0.1,
                "alpha": 1.2,
                "beta": 0.6,
                "min_authority": 0.15,
            }],
            remappings=[
                ("fmu/out/vehicle_odometry", "/twinguard/replay/vehicle_odometry"),
            ],
        ),
        Node(
            package="twinguard_swarm_integrity_cpp",
            executable="formation_supervisor_node",
            name="formation_supervisor_node",
            namespace="twinguard",
            output="screen",
            parameters=[{
                "drone_id": 0,
                "target_system": 1,
                "nominal_velocity_limit_mps": 3.0,
                "degraded_threshold": 0.5,
                "min_authority_scale": 0.15,
                "stale_timeout_ms": 2000,
                "setpoint_rate_hz": 10.0,
                "auto_arm": False,
                "force_arm": False,
                "mission_mode": "hold",
                "static_obstacle_enabled": False,
            }],
        ),
    ])
