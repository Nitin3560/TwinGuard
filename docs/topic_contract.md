# ROS 2 Topic Contract

This document defines the logical ROS 2 communication interfaces used throughout TwinGuard.

Rather than documenting implementation details alone, this contract specifies how estimation, planning, supervision, and PX4 exchange information during runtime.

The logical interfaces remain stable even if individual ROS namespaces, launch configurations, or PX4 versions change.

Implementation-specific topic bindings are documented separately for reference.

---

# Design Philosophy

TwinGuard follows a simple communication model.

- Every subsystem publishes a clearly defined responsibility.
- Components communicate only through ROS 2 topics.
- Runtime interfaces remain stable while implementations evolve.
- Estimation, planning, supervision, and navigation remain loosely coupled.

The most important interface within the system is:

```text
trust_state
```

Every downstream autonomy component consumes the same localization integrity estimate instead of computing its own confidence.

---

# PX4 Inputs

These topics provide the primary vehicle state consumed by the TwinGuard autonomy pipeline.

## Logical Interface

```text
/drone_i/fmu/out/vehicle_odometry
/drone_i/fmu/out/vehicle_local_position
/drone_i/fmu/out/vehicle_status
/drone_i/fmu/out/sensor_gps
/drone_i/fmu/out/vehicle_imu
```

### Expected Message Types

```text
px4_msgs/msg/VehicleOdometry
px4_msgs/msg/VehicleLocalPosition
px4_msgs/msg/VehicleStatus
px4_msgs/msg/SensorGps
px4_msgs/msg/VehicleImu
```

### Current Implementation

```text
/drone_0/fmu/out/vehicle_odometry -> integrity_node_cpp
/drone_0/fmu/out/vehicle_odometry -> formation_supervisor_node

/twinguard/replay/vehicle_odometry -> integrity_node_cpp
/twinguard/replay/vehicle_odometry -> formation_supervisor_node

/drone_2/twinguard/replay/vehicle_odometry -> integrity_node_cpp
/drone_2/twinguard/replay/vehicle_odometry -> formation_supervisor_node

/drone_0/fmu/out/vehicle_odometry -> ekf_integrity_node
```

---

# Camera and Visual Odometry

Visual odometry provides an additional localization source for the optional Kalman-based integrity pipeline.

## Logical Interface

```text
/drone_0/twinguard/camera/image_raw
/drone_0/twinguard/camera/depth
/drone_0/twinguard/visual_odometry
/drone_0/twinguard/visual_odometry_diagnostics
```

### Expected Message Types

```text
sensor_msgs/msg/Image
geometry_msgs/msg/TwistStamped
diagnostic_msgs/msg/DiagnosticArray
```

### Current Implementation

```text
Gazebo x500_depth image      -> ros_gz_image -> image_raw
Gazebo x500_depth depth      -> ros_gz_image -> depth

image_raw                    -> visual_odometry_node
depth                        -> visual_odometry_node

visual_odometry              <- visual_odometry_node
visual_odometry_diagnostics  -> ekf_integrity_node
```

The `visual_odometry_diagnostics` topic combines:

- tracking quality,
- tracked feature count,
- tracking error,
- estimated velocity,

into a single `DiagnosticArray` message, allowing the EKF integrity node to scale visual measurement uncertainty using one synchronized interface.

---

# PX4 Outputs

The Formation Supervisor is the only component responsible for publishing offboard commands to PX4.

## Logical Interface

```text
/drone_i/fmu/in/offboard_control_mode
/drone_i/fmu/in/trajectory_setpoint
/drone_i/fmu/in/vehicle_command
```

### Expected Message Types

```text
px4_msgs/msg/OffboardControlMode
px4_msgs/msg/TrajectorySetpoint
px4_msgs/msg/VehicleCommand
```

### Current Implementation

```text
/drone_0/fmu/in/offboard_control_mode -> formation_supervisor_node
/drone_0/fmu/in/trajectory_setpoint   -> formation_supervisor_node
/drone_0/fmu/in/vehicle_command       -> formation_supervisor_node

/fmu/in/offboard_control_mode         -> formation_supervisor_node
/fmu/in/trajectory_setpoint           -> formation_supervisor_node
/fmu/in/vehicle_command               -> formation_supervisor_node

/px4_1/fmu/in/offboard_control_mode   -> formation_supervisor_node
/px4_1/fmu/in/trajectory_setpoint     -> formation_supervisor_node
/px4_1/fmu/in/vehicle_command         -> formation_supervisor_node

/px4_2/fmu/in/offboard_control_mode   -> formation_supervisor_node
/px4_2/fmu/in/trajectory_setpoint     -> formation_supervisor_node
/px4_2/fmu/in/vehicle_command         -> formation_supervisor_node
```

No planner or navigation component publishes directly to PX4.

All commands pass through the Formation Supervisor, where authority scaling and integrity constraints are enforced.

---

# TwinGuard Runtime Topics

These topics expose localization integrity and supervisory state to the remainder of the autonomy stack.

## Logical Interface

```text
/twinguard/drone_i/trust_state
/twinguard/drone_i/integrity_diagnostics
/twinguard/drone_i/supervisor_diagnostics
/twinguard/swarm/formation_error
/twinguard/swarm/mission_status
```

### Current Implementation

```text
integrity_node_cpp
    ├── integrity_diagnostics
    └── trust_state

ekf_integrity_node
    ├── integrity_diagnostics
    └── trust_state

formation_supervisor_node
    └── supervisor_diagnostics
```

---

# Nav2 Integration

TwinGuard extends Nav2 using the existing trust interface.

The following components subscribe to `trust_state`:

```text
IsAgentTrustworthy
Behavior Tree Condition

TwinGuardIntegrityLayer
Nav2 Costmap Layer
```

Both plugins consume the existing

```text
geometry_msgs/msg/PointStamped
```

contract, where

```text
point.z = authority_scale
```

No additional Nav2-specific TwinGuard topics are introduced.

---

# Logging Schema

Experiment logs follow a common CSV schema to simplify offline analysis and comparison across scenarios.

```text
time
drone_id
scenario

gt_x
gt_y
gt_z

odom_x
odom_y
odom_z

twin_x
twin_y
twin_z

residual

trust

authority_scale

fault_active

formation_error

mission_phase
```

This schema is shared across replay experiments, live PX4 SITL validation, and future multi-UAV evaluations to provide consistent post-processing and visualization.
