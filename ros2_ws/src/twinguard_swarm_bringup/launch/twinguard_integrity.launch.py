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
                package="twinguard_swarm_integrity",
                executable="formation_supervisor",
                name="formation_supervisor",
                namespace="twinguard",
                output="screen",
                parameters=[{"num_drones": 3}],
            ),
        ]
    )
