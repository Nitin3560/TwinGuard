# Engineering Plan

This document defines the implementation roadmap for TwinGuard-Swarm-Gazebo. The objective is to build a reproducible simulation pipeline that connects PX4 SITL, Gazebo, ROS 2, digital-twin prediction, trust-aware integrity monitoring, and formation-level supervision.

## Target Components

- PX4 SITL autopilot
- Gazebo simulator
- ROS 2 nodes and topics
- px4_msgs message types
- C++17 integrity scoring module
- rosbag2 or CSV logging
- Digital-twin prediction and trust-management logic

## Core ROS 2 Nodes

### `formation_supervisor`

Inputs:

- per-UAV odometry
- trust scores
- mission phase

Outputs:

- desired setpoints
- authority scale per UAV
- reconfiguration status

Responsibilities:

- Maintain nominal formation geometry.
- Reduce command authority for low-trust agents.
- Reassign formation roles after an isolation decision.

### `digital_twin_node`

Inputs:

- vehicle odometry
- previous setpoint/control

Outputs:

- predicted state
- expected GPS/IMU proxy

Responsibilities:

- Propagate expected per-UAV motion from recent state and command history.
- Provide a reference state for innovation and residual computation.

### `integrity_node`

Inputs:

- measured state/sensor proxy
- digital twin prediction

Outputs:

- residual
- NIS-like normalized innovation
- trust score
- fault label

Responsibilities:

- Compare measured state against digital-twin prediction.
- Convert residual statistics into trust and fault labels.
- Publish integrity diagnostics for the supervisor and logger.
- Own the latency-sensitive scoring path in C++.

### `attack_injector`

Inputs:

- clean sensor/state stream

Outputs:

- corrupted stream for selected UAV/scenario

Scenarios:

- GPS spoofing
- position drift
- communication dropout
- delayed/replay state

Responsibilities:

- Provide reproducible attacks with seed-controlled timing and magnitude.
- Preserve clean logs for baseline comparison.

### `logger`

Outputs:

- CSV logs
- rosbag2 recording
- metrics summary

Responsibilities:

- Record state, trust, attack, and formation data.
- Generate experiment metrics used by the command-center replay.

## Minimal Experiment Matrix

```text
nominal
gps_spoof_drone_3
comm_dropout_drone_3
replay_state_drone_3
combined_gps_comm
```

Seeds:

```text
0 1 2 3 4
```

Metrics:

- swarm RMSE
- attacked UAV residual
- trust detection time
- false alarm rate
- recovery time
- connectivity/formation error
- mission success

## Milestones

1. Build ROS 2 packages and launch files.
2. Validate one PX4 `gz_x500` vehicle publishing `VehicleOdometry` into the C++ integrity node.
3. Bind additional integrity inputs to PX4 GPS, IMU, and vehicle status topics.
4. Add deterministic attack injection for GPS spoofing and replay.
5. Publish trust and fault diagnostics per UAV.
6. Generate trust-aware formation setpoints.
7. Record experiment logs and compute metrics.
8. Generate command-center replay from recorded logs.
