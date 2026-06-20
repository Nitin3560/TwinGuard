from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _integrity_node(drone_id, namespace, odometry_topic):
    return Node(
        package="twinguard_swarm_integrity_cpp",
        executable="integrity_node_cpp",
        name=f"integrity_drone_{drone_id}_cpp",
        namespace=namespace,
        output="screen",
        parameters=[
            {
                "drone_id": drone_id,
                "stale_timeout_ms": 500,
                "prediction_dt": 0.1,
            }
        ],
        remappings=[
            ("fmu/out/vehicle_odometry", odometry_topic),
        ],
    )


def _mission_controller(
    drone_id,
    namespace,
    px4_prefix,
    trust_topic,
    center_x,
    center_y,
    altitude,
    radius,
):
    return Node(
        package="twinguard_swarm_integrity",
        executable="offboard_mission_controller",
        name=f"offboard_mission_drone_{drone_id}",
        namespace=namespace,
        output="screen",
        parameters=[
            {
                "drone_id": drone_id,
                "px4_prefix": px4_prefix,
                "trust_topic": trust_topic,
                "takeoff_altitude_m": altitude,
                "mission_radius_m": radius,
                "mission_period_s": 22.0,
                "center_x_m": center_x,
                "center_y_m": center_y,
                "min_authority_scale": 0.25,
                "setpoint_rate_hz": 20.0,
                "auto_arm": True,
                "force_arm": True,
                "mission_mode": "circle",
            }
        ],
    )


def generate_launch_description():
    dataset_csv = LaunchConfiguration("dataset_csv")
    error_scale = LaunchConfiguration("error_scale")
    max_offset_m = LaunchConfiguration("max_offset_m")
    replay_rate_hz = LaunchConfiguration("replay_rate_hz")
    mission_radius_m = LaunchConfiguration("mission_radius_m")
    takeoff_altitude_m = LaunchConfiguration("takeoff_altitude_m")
    drone_0_prefix = LaunchConfiguration("drone_0_px4_prefix")
    drone_1_prefix = LaunchConfiguration("drone_1_px4_prefix")
    drone_2_prefix = LaunchConfiguration("drone_2_px4_prefix")

    drone_0_odom = LaunchConfiguration("drone_0_odometry")
    drone_1_odom = LaunchConfiguration("drone_1_odometry")
    drone_2_odom = LaunchConfiguration("drone_2_odometry")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "dataset_csv",
                default_value="/home/nitin/scenario23_seq20_real_channel_timeseries.csv",
            ),
            DeclareLaunchArgument("error_scale", default_value="0.1"),
            DeclareLaunchArgument("max_offset_m", default_value="3.0"),
            DeclareLaunchArgument("replay_rate_hz", default_value="20.0"),
            DeclareLaunchArgument("mission_radius_m", default_value="3.0"),
            DeclareLaunchArgument("takeoff_altitude_m", default_value="2.0"),
            # PX4 multi-vehicle namespaces vary by setup. These defaults match
            # the common PX4 ROS 2 SITL layout: first vehicle unprefixed, then
            # px4_1, px4_2 for later instances.
            DeclareLaunchArgument("drone_0_px4_prefix", default_value=""),
            DeclareLaunchArgument("drone_1_px4_prefix", default_value="px4_1"),
            DeclareLaunchArgument("drone_2_px4_prefix", default_value="px4_2"),
            DeclareLaunchArgument("drone_0_odometry", default_value="/fmu/out/vehicle_odometry"),
            DeclareLaunchArgument("drone_1_odometry", default_value="/px4_1/fmu/out/vehicle_odometry"),
            DeclareLaunchArgument("drone_2_odometry", default_value="/px4_2/fmu/out/vehicle_odometry"),
            Node(
                package="twinguard_dataset_replay",
                executable="dataset_replay_node",
                name="dataset_replay_drone_2",
                namespace="drone_2/twinguard",
                output="screen",
                parameters=[
                    {
                        "dataset_csv": dataset_csv,
                        "replay_rate_hz": replay_rate_hz,
                        "error_scale": error_scale,
                        "max_offset_m": max_offset_m,
                        "loop": True,
                        "perturb_axis": "x",
                    }
                ],
                remappings=[
                    ("input_vehicle_odometry", drone_2_odom),
                    (
                        "output_vehicle_odometry",
                        "/drone_2/twinguard/replay/vehicle_odometry",
                    ),
                ],
            ),
            _integrity_node(
                0,
                "drone_0/twinguard",
                drone_0_odom,
            ),
            _integrity_node(
                1,
                "drone_1/twinguard",
                drone_1_odom,
            ),
            _integrity_node(
                2,
                "drone_2/twinguard",
                "/drone_2/twinguard/replay/vehicle_odometry",
            ),
            _mission_controller(
                0,
                "drone_0/twinguard",
                drone_0_prefix,
                "/drone_0/twinguard/trust_state",
                0.0,
                0.0,
                takeoff_altitude_m,
                mission_radius_m,
            ),
            _mission_controller(
                1,
                "drone_1/twinguard",
                drone_1_prefix,
                "/drone_1/twinguard/trust_state",
                -1.8,
                -1.2,
                takeoff_altitude_m,
                mission_radius_m,
            ),
            _mission_controller(
                2,
                "drone_2/twinguard",
                drone_2_prefix,
                "/drone_2/twinguard/trust_state",
                -1.8,
                1.2,
                takeoff_altitude_m,
                mission_radius_m,
            ),
        ]
    )
