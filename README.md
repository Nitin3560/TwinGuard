# TwinGuard-Swarm-Gazebo

**Digital Twin-Assisted Resilient UAV Swarm Autonomy with ROS 2, Gazebo, and PX4 SITL**

TwinGuard-Swarm-Gazebo is a simulation and autonomy framework for evaluating resilient UAV swarm coordination under sensor degradation and adversarial attacks. The system combines PX4 software-in-the-loop flight dynamics, Gazebo-based simulation, ROS 2 middleware, digital-twin state prediction, trust-aware integrity monitoring, and formation-level supervisory control.

The project is designed to support repeatable experiments where a UAV swarm performs a coordinated mission while one or more agents experience GPS spoofing, communication degradation, delayed state replay, or sensing faults. TwinGuard estimates per-agent integrity, reduces authority for compromised agents, and reconfigures the formation to preserve mission-level performance.

## Key Capabilities

- PX4 SITL multicopter simulation in Gazebo
- ROS 2 integration for state, command, diagnostics, and logging
- C++ integrity scoring and trust-management core
- Multi-UAV formation supervision
- Per-agent digital twin prediction
- Residual and normalized-innovation-based anomaly scoring
- Stateful trust management and authority scaling
- GPS spoofing, communication dropout, and replay-attack scenarios
- Experiment logging for RMSE, recovery time, trust, residuals, and mission success
- Command-center replay generated from experiment logs

## System Architecture

```text
PX4 SITL + Gazebo
        |
        v
ROS 2 / PX4 Bridge
        |
        +--> Vehicle odometry
        +--> GPS / IMU / status streams
        +--> Offboard setpoint commands
        |
        v
TwinGuard Autonomy Layer
        |
        +--> Digital Twin Predictor
        +--> Integrity Engine
        +--> Trust Manager
        +--> Attack Injector
        +--> Formation Supervisor
        |
        v
Experiment Logs + Command-Center Replay
```

Detailed architecture notes are available in [docs/architecture.md](docs/architecture.md).

## Repository Layout

```text
TwinGuard-Swarm-Gazebo/
├── docs/
│   ├── architecture.md
│   ├── engineering_plan.md
│   ├── environment.md
│   ├── quickstart.md
│   └── topic_contract.md
├── ros2_ws/
│   └── src/
│       ├── twinguard_swarm_bringup/
│       ├── twinguard_swarm_integrity_cpp/
│       └── twinguard_swarm_integrity/
├── experiments/
├── visualization/
└── results/
```

## Target Experiment

The reference experiment uses a multi-UAV formation mission:

1. Spawn a PX4/Gazebo multicopter swarm.
2. Fly a nominal formation trajectory.
3. Inject GPS spoofing or communication degradation into a selected UAV.
4. Predict expected state using a per-agent digital twin.
5. Compute residuals and trust scores.
6. Reduce authority of the compromised UAV.
7. Reconfigure the formation.
8. Report tracking RMSE, detection time, recovery time, and mission success.

## ROS 2 Integration

Planned nodes and packages:

- `twinguard_swarm_integrity_cpp`: C++ package for digital-twin integrity scoring, trust, fault labels, and authority scaling.
- `integrity_node_cpp`: C++ ROS 2 node that subscribes to PX4 `VehicleOdometry`, predicts expected state, and publishes residual/trust diagnostics.
- `digital_twin_node`: predicts per-UAV expected state.
- `formation_supervisor`: generates trust-aware formation setpoints.
- `attack_injector`: injects reproducible sensor/communication faults.
- `experiment_logger`: records CSV and rosbag2 outputs.

The ROS 2 package skeleton is located under [ros2_ws/src](ros2_ws/src). The latency-sensitive integrity/scoring path is implemented in C++; Python is reserved for launch-time orchestration, experiment tooling, and rapid prototyping.

## Simulation Stack

Recommended stack:

```text
Ubuntu 22.04/24.04
ROS 2 Humble or Jazzy
Gazebo / Gazebo Harmonic-compatible PX4 setup
PX4-Autopilot SITL
px4_msgs
px4_ros_com
Micro XRCE-DDS Agent
C++17
```

Official references:

- [PX4 Gazebo Simulation](https://docs.px4.io/main/en/sim_gazebo_gz/)
- [PX4 ROS 2 User Guide](https://docs.px4.io/main/en/ros2/user_guide)
- [PX4 ROS 2 Offboard Control](https://docs.px4.io/main/en/ros2/offboard_control)

## Implementation Status

This repository defines the ROS 2 package structure, autonomy-layer interfaces, topic contract, setup plan, C++ integrity-scoring package, and initial supervisor node scaffolding. The C++ integrity node includes the first live PX4 data path by subscribing to `px4_msgs/msg/VehicleOdometry` and publishing trust/residual diagnostics. The next engineering milestone is to validate that path against one PX4 `gz_x500` SITL vehicle, then connect supervisor logic to PX4 offboard-control topics.

## Intended Outcome

The intended final artifact is a reproducible UAV swarm simulation pipeline where dashboard values, metrics, and command-center videos are generated from PX4/Gazebo/ROS 2 experiment logs.
