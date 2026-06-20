# ROS 2 Topic Contract

This document defines the intended ROS 2 topic contract for TwinGuard-Swarm-Gazebo before final binding to exact PX4 namespace and message configuration.

PX4 topic names vary by namespace and version. The implementation should map these logical topics to `px4_msgs` types.

## Inputs From PX4/Gazebo

```text
/drone_i/fmu/out/vehicle_odometry
/drone_i/fmu/out/vehicle_local_position
/drone_i/fmu/out/vehicle_status
/drone_i/fmu/out/sensor_gps
/drone_i/fmu/out/vehicle_imu
```

Expected PX4 message families:

```text
px4_msgs/msg/VehicleOdometry
px4_msgs/msg/VehicleLocalPosition
px4_msgs/msg/VehicleStatus
px4_msgs/msg/SensorGps
px4_msgs/msg/VehicleImu
```

Current implemented binding:

```text
/drone_0/fmu/out/vehicle_odometry -> integrity_node_cpp
/drone_0/fmu/out/vehicle_odometry -> formation_supervisor_node
```

## Commands To PX4

```text
/drone_i/fmu/in/offboard_control_mode
/drone_i/fmu/in/trajectory_setpoint
/drone_i/fmu/in/vehicle_command
```

Expected PX4 message families:

```text
px4_msgs/msg/OffboardControlMode
px4_msgs/msg/TrajectorySetpoint
px4_msgs/msg/VehicleCommand
```

Current implemented binding:

```text
/drone_0/fmu/in/offboard_control_mode -> formation_supervisor_node
/drone_0/fmu/in/trajectory_setpoint   -> formation_supervisor_node
/drone_0/fmu/in/vehicle_command       -> formation_supervisor_node
```

## TwinGuard Diagnostics

```text
/twinguard/drone_i/predicted_state
/twinguard/drone_i/residual
/twinguard/drone_i/trust
/twinguard/drone_i/fault_label
/twinguard/swarm/formation_error
/twinguard/swarm/mission_status
```

Current implemented binding:

```text
/drone_0/twinguard/integrity_diagnostics  <- integrity_node_cpp
/drone_0/twinguard/trust_state            <- integrity_node_cpp
/drone_0/twinguard/supervisor_diagnostics <- formation_supervisor_node
```

## Logging Schema

CSV columns:

```text
time
drone_id
scenario
gt_x gt_y gt_z
odom_x odom_y odom_z
twin_x twin_y twin_z
residual
trust
authority_scale
fault_active
formation_error
mission_phase
```
