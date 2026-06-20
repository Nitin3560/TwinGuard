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
- Real dataset replay into live PX4/Gazebo odometry for video-recordable validation

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
│   ├── real_dataset_replay.md
│   └── topic_contract.md
├── ros2_ws/
│   └── src/
│       ├── twinguard_swarm_bringup/
│       ├── twinguard_dataset_replay/
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
- `dataset_replay_node`: Python ROS 2 bridge that applies a real dataset degradation profile to live PX4 odometry for validation and recording.
- `offboard_mission_controller`: ROS 2 PX4 offboard controller that publishes `OffboardControlMode`, `TrajectorySetpoint`, and `VehicleCommand` messages for repeatable mission flight.
- `digital_twin_node`: predicts per-UAV expected state.
- `formation_supervisor`: generates trust-aware formation setpoints.
- `attack_injector`: injects reproducible sensor/communication faults.
- `experiment_logger`: records CSV and rosbag2 outputs.

The ROS 2 package skeleton is located under [ros2_ws/src](ros2_ws/src). The latency-sensitive integrity/scoring path is implemented in C++; Python is reserved for launch-time orchestration, experiment tooling, dataset replay, and mission-control prototyping.

## Single-UAV PX4 Validation Run

The current live validation path runs one PX4 `gz_x500` vehicle, replays a real failure/degradation dataset into the odometry stream, scores the perturbed state with the C++ TwinGuard integrity node, and flies a repeatable offboard mission in Gazebo.

Terminal 1:

```bash
cd ~/PX4-Autopilot
make px4_sitl gz_x500
```

Terminal 2:

```bash
cd ~/Micro-XRCE-DDS-Agent/build
MicroXRCEAgent udp4 -p 8888
```

Terminal 3:

```bash
cd ~/px4_ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch twinguard_swarm_bringup twinguard_single_uav_replay_mission.launch.py \
  dataset_csv:=/home/nitin/scenario23_seq20_real_channel_timeseries.csv \
  error_scale:=0.1 \
  max_offset_m:=3.0 \
  mission_radius_m:=3.0 \
  takeoff_altitude_m:=2.0
```

The real dataset is used as a replayed degradation profile for sensor/integrity validation. It does not replace the PX4 flight controller or directly define the UAV path. The offboard mission controller owns the flight trajectory; TwinGuard owns the integrity/trust response.

## Three-UAV Formation Validation Run

The next validation target is a lightweight swarm run with three PX4 `x500` vehicles:

```text
Drone 0: nominal leader-style mission controller
Drone 1: nominal formation follower
Drone 2: dataset-degraded UAV with TwinGuard replay/integrity scoring
```

The formation uses three synchronized offboard mission controllers with fixed spatial offsets. Drone 2 receives the real dataset replay on its integrity path so the video shows a visible swarm mission while the diagnostics show degradation, residual growth, and trust response for the affected vehicle.

Start a three-vehicle PX4/Gazebo run using the multi-vehicle launch method supported by your PX4 checkout. Then confirm the generated ROS 2 odometry topics:

```bash
ros2 topic list | grep vehicle_odometry
```

The default launch arguments expect the common PX4 ROS 2 topic layout:

```text
/fmu/out/vehicle_odometry
/px4_1/fmu/out/vehicle_odometry
/px4_2/fmu/out/vehicle_odometry
```

Launch the TwinGuard formation mission:

```bash
cd ~/px4_ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch twinguard_swarm_bringup twinguard_three_uav_replay_mission.launch.py \
  dataset_csv:=/home/nitin/scenario23_seq20_real_channel_timeseries.csv
```

If your PX4 topics use different prefixes, override them explicitly:

```bash
ros2 launch twinguard_swarm_bringup twinguard_three_uav_replay_mission.launch.py \
  dataset_csv:=/home/nitin/scenario23_seq20_real_channel_timeseries.csv \
  drone_0_px4_prefix:="" \
  drone_1_px4_prefix:=px4_1 \
  drone_2_px4_prefix:=px4_2 \
  drone_0_odometry:=/fmu/out/vehicle_odometry \
  drone_1_odometry:=/px4_1/fmu/out/vehicle_odometry \
  drone_2_odometry:=/px4_2/fmu/out/vehicle_odometry
```

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

This repository defines the ROS 2 package structure, autonomy-layer interfaces, topic contract, setup plan, C++ integrity-scoring package, real dataset replay bridge, and an initial PX4 offboard mission controller. The current validated path is a single PX4 `gz_x500` vehicle with live odometry, dataset replay, C++ trust scoring, and offboard mission setpoints. The next engineering milestone is to extend the launch stack to multiple PX4 vehicle namespaces and promote the single-vehicle mission controller into a trust-aware swarm formation controller.

## Intended Outcome

The intended final artifact is a reproducible UAV swarm simulation pipeline where dashboard values, metrics, and command-center videos are generated from PX4/Gazebo/ROS 2 experiment logs.
