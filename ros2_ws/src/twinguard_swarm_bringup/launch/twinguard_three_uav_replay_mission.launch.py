from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
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


def _formation_supervisor(
    drone_id,
    namespace,
    odometry_topic,
    offboard_control_mode_topic,
    trajectory_setpoint_topic,
    vehicle_command_topic,
    center_x,
    center_y,
    nominal_z,
    radius,
):
    return Node(
        package="twinguard_swarm_integrity_cpp",
        executable="formation_supervisor_node",
        name=f"formation_supervisor_drone_{drone_id}",
        namespace=namespace,
        output="screen",
        parameters=[
            {
                "drone_id": drone_id,
                "target_system": drone_id + 1,
                "nominal_z_m": nominal_z,
                "mission_radius_m": radius,
                "mission_period_s": 22.0,
                "center_x_m": center_x,
                "center_y_m": center_y,
                "min_authority_scale": 0.25,
                "nominal_velocity_limit_mps": 3.0,
                "degraded_threshold": 0.5,
                "setpoint_rate_hz": 20.0,
                "auto_arm": True,
                "force_arm": True,
                "mission_mode": "circle",
            }
        ],
        remappings=[
            ("fmu/out/vehicle_odometry", odometry_topic),
            ("fmu/in/offboard_control_mode", offboard_control_mode_topic),
            ("fmu/in/trajectory_setpoint", trajectory_setpoint_topic),
            ("fmu/in/vehicle_command", vehicle_command_topic),
        ],
    )


def generate_launch_description():
    dataset_csv = LaunchConfiguration("dataset_csv")
    error_scale = LaunchConfiguration("error_scale")
    max_offset_m = LaunchConfiguration("max_offset_m")
    replay_rate_hz = LaunchConfiguration("replay_rate_hz")
    mission_radius_m = LaunchConfiguration("mission_radius_m")
    takeoff_altitude_m = LaunchConfiguration("takeoff_altitude_m")
    nominal_z_m = PythonExpression(["-", takeoff_altitude_m])

    drone_0_odom = LaunchConfiguration("drone_0_odometry")
    drone_1_odom = LaunchConfiguration("drone_1_odometry")
    drone_2_odom = LaunchConfiguration("drone_2_odometry")
    drone_0_offboard = LaunchConfiguration("drone_0_offboard_control_mode")
    drone_1_offboard = LaunchConfiguration("drone_1_offboard_control_mode")
    drone_2_offboard = LaunchConfiguration("drone_2_offboard_control_mode")
    drone_0_setpoint = LaunchConfiguration("drone_0_trajectory_setpoint")
    drone_1_setpoint = LaunchConfiguration("drone_1_trajectory_setpoint")
    drone_2_setpoint = LaunchConfiguration("drone_2_trajectory_setpoint")
    drone_0_command = LaunchConfiguration("drone_0_vehicle_command")
    drone_1_command = LaunchConfiguration("drone_1_vehicle_command")
    drone_2_command = LaunchConfiguration("drone_2_vehicle_command")

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
            # Defaults match the common PX4 multi-vehicle ROS 2 layout.
            DeclareLaunchArgument("drone_0_odometry", default_value="/fmu/out/vehicle_odometry"),
            DeclareLaunchArgument("drone_1_odometry", default_value="/px4_1/fmu/out/vehicle_odometry"),
            DeclareLaunchArgument("drone_2_odometry", default_value="/px4_2/fmu/out/vehicle_odometry"),
            DeclareLaunchArgument(
                "drone_0_offboard_control_mode",
                default_value="/fmu/in/offboard_control_mode",
            ),
            DeclareLaunchArgument(
                "drone_1_offboard_control_mode",
                default_value="/px4_1/fmu/in/offboard_control_mode",
            ),
            DeclareLaunchArgument(
                "drone_2_offboard_control_mode",
                default_value="/px4_2/fmu/in/offboard_control_mode",
            ),
            DeclareLaunchArgument(
                "drone_0_trajectory_setpoint",
                default_value="/fmu/in/trajectory_setpoint",
            ),
            DeclareLaunchArgument(
                "drone_1_trajectory_setpoint",
                default_value="/px4_1/fmu/in/trajectory_setpoint",
            ),
            DeclareLaunchArgument(
                "drone_2_trajectory_setpoint",
                default_value="/px4_2/fmu/in/trajectory_setpoint",
            ),
            DeclareLaunchArgument("drone_0_vehicle_command", default_value="/fmu/in/vehicle_command"),
            DeclareLaunchArgument(
                "drone_1_vehicle_command",
                default_value="/px4_1/fmu/in/vehicle_command",
            ),
            DeclareLaunchArgument(
                "drone_2_vehicle_command",
                default_value="/px4_2/fmu/in/vehicle_command",
            ),
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
            _formation_supervisor(
                0,
                "drone_0/twinguard",
                drone_0_odom,
                drone_0_offboard,
                drone_0_setpoint,
                drone_0_command,
                0.0,
                0.0,
                nominal_z_m,
                mission_radius_m,
            ),
            _formation_supervisor(
                1,
                "drone_1/twinguard",
                drone_1_odom,
                drone_1_offboard,
                drone_1_setpoint,
                drone_1_command,
                -1.8,
                -1.2,
                nominal_z_m,
                mission_radius_m,
            ),
            _formation_supervisor(
                2,
                "drone_2/twinguard",
                "/drone_2/twinguard/replay/vehicle_odometry",
                drone_2_offboard,
                drone_2_setpoint,
                drone_2_command,
                -1.8,
                1.2,
                nominal_z_m,
                mission_radius_m,
            ),
        ]
    )
