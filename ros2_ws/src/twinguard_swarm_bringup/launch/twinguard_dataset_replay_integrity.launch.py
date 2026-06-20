from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    dataset_csv = LaunchConfiguration("dataset_csv")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "dataset_csv",
                description="Path to real dataset CSV used for degradation replay.",
            ),
            DeclareLaunchArgument("error_scale", default_value="0.1"),
            DeclareLaunchArgument("max_offset_m", default_value="3.0"),
            Node(
                package="twinguard_dataset_replay",
                executable="dataset_replay_node",
                name="dataset_replay",
                namespace="twinguard",
                output="screen",
                parameters=[
                    {
                        "dataset_csv": dataset_csv,
                        "replay_rate_hz": 20.0,
                        "error_scale": LaunchConfiguration("error_scale"),
                        "max_offset_m": LaunchConfiguration("max_offset_m"),
                        "loop": True,
                        "perturb_axis": "x",
                    }
                ],
                remappings=[
                    ("input_vehicle_odometry", "/fmu/out/vehicle_odometry"),
                    ("output_vehicle_odometry", "/twinguard/replay/vehicle_odometry"),
                ],
            ),
            Node(
                package="twinguard_swarm_integrity_cpp",
                executable="integrity_node_cpp",
                name="integrity_drone_0_cpp",
                namespace="twinguard",
                output="screen",
                parameters=[
                    {
                        "drone_id": 0,
                        "stale_timeout_ms": 500,
                        "prediction_dt": 0.1,
                    }
                ],
                remappings=[
                    ("fmu/out/vehicle_odometry", "/twinguard/replay/vehicle_odometry"),
                ],
            ),
        ]
    )
