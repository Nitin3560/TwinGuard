# TwinGuard-Swarm-Gazebo

**Digital Twin-Assisted Resilient UAV Swarm Autonomy with ROS 2, Gazebo, and PX4 SITL**

TwinGuard-Swarm-Gazebo is a simulation and autonomy framework for evaluating resilient UAV swarm coordination under sensor degradation and adversarial attacks. The system combines PX4 software-in-the-loop flight dynamics, Gazebo-based simulation, ROS 2 middleware, digital-twin state prediction, trust-aware integrity monitoring, and formation-level supervisory control.

The project is designed to support repeatable experiments where a UAV swarm performs a coordinated mission while one or more agents experience GPS spoofing, communication degradation, delayed state replay, or sensing faults. TwinGuard estimates per-agent integrity, reduces authority for compromised agents, and reconfigures the formation to preserve mission-level performance.

## Key Capabilities

- PX4 SITL multicopter simulation in Gazebo
- ROS 2 integration for state, command, diagnostics, and logging
- C++ integrity scoring and trust-management core
- BehaviorTree.CPP mission supervision for hold, reroute, and nominal flight branches
- Local A* replanning over a coarse static 3D voxel grid for known no-fly volumes
- Optional EKF-based multi-source estimator fusing PX4 odometry with sparse optical-flow visual odometry
- Nav2-facing BT condition and costmap layer plugins for trust-aware navigation integration
- Deployment-oriented Dockerfiles and Fast-DDS discovery-server compose profile
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
│   ├── deployment.md
│   ├── engineering_plan.md
│   ├── environment.md
│   ├── quickstart.md
│   ├── real_dataset_replay.md
│   └── topic_contract.md
├── docker/
│   ├── px4_sitl/
│   └── ros2_ws/
├── docker-compose.microservices.yaml
├── docker-compose.yaml
├── ros2_ws/
│   └── src/
│       ├── twinguard_swarm_bringup/
│       ├── twinguard_dataset_replay/
│       ├── twinguard_swarm_estimation_cpp/
│       ├── twinguard_swarm_nav2_cpp/
│       ├── twinguard_swarm_planning_cpp/
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

Implemented and planned nodes/packages:

- `twinguard_swarm_integrity_cpp`: C++ package for digital-twin integrity scoring, trust, fault labels, and authority scaling.
- `integrity_node_cpp`: C++ ROS 2 node that subscribes to PX4 `VehicleOdometry`, predicts expected state, and publishes residual/trust diagnostics.
- `formation_supervisor_node`: C++ ROS 2 node that subscribes to TwinGuard trust/diagnostics and PX4 odometry, generates static-hold or circular mission setpoints, then publishes trust-gated `OffboardControlMode`, `TrajectorySetpoint`, and `VehicleCommand` messages.
- `twinguard_swarm_estimation_cpp`: C++ package with a 6-state EKF, sparse optical-flow visual odometry, and an opt-in EKF integrity node.
- `ekf_integrity_node`: alternative integrity node that fuses PX4 position updates and quality-scored visual-odometry velocity updates, then publishes the same `trust_state` contract consumed by the supervisor.
- `visual_odometry_node`: converts bridged Gazebo camera/depth images into per-feature-depth-scaled optical-flow velocity estimates plus atomic quality/velocity diagnostics.
- `twinguard_swarm_nav2_cpp`: Nav2-facing plugin package with a BT condition node and a costmap layer that consume TwinGuard trust.
- `IsAgentTrustworthy`: Nav2 BT condition plugin that fails closed unless `authority_scale` from `trust_state` is above the configured threshold.
- `TwinGuardIntegrityLayer`: Nav2 costmap layer plugin that encodes localization self-trust as planning conservatism around the robot's own position, distinct from obstacle or sensor layers.
- `twinguard_swarm_planning_cpp`: C++ package with BehaviorTree.CPP leaves and a pure A* local planner for static obstacle/no-fly-volume rerouting.
- `dataset_replay_node`: Python ROS 2 bridge that applies a real dataset degradation profile to live PX4 odometry for validation and recording.
- `digital_twin_node`: predicts per-UAV expected state.
- `attack_injector`: injects reproducible sensor/communication faults.
- `experiment_logger`: records CSV and rosbag2 outputs.

The ROS 2 package skeleton is located under [ros2_ws/src](ros2_ws/src). The latency-sensitive integrity/scoring and trust-gated offboard-supervision paths are implemented in C++; Python is reserved for launch-time orchestration, experiment tooling, dataset replay, and mission-control prototyping.
A single trust-gated supervisor (`formation_supervisor_node`) handles both static-hold and circular-mission modes, validated against both live PX4 SITL odometry and real-dataset-replay-perturbed odometry.
Phase 2 adds an explicit Behavior Tree above the offboard supervisor: suspected attacks publish a hold setpoint, blocked straight-line paths invoke A* rerouting around configured static obstacles, and otherwise the nominal mission setpoint is used. The final `OffboardSupervisor::step()` authority gate still decides whether to pass through, slow down, or hard-hold.
The local A* planner is intentionally scoped to short-horizon rerouting: with the default 0.5 m cell size and 32-cell extent, it solves roughly 32 m start-to-goal spans per axis around the current query. Larger or unreachable local plans fall back to the nominal branch rather than being claimed as global planning.
Phase 3 adds an opt-in EKF path through `twinguard_ekf_integrity.launch.py`: launch PX4 with `gz_x500_depth`, bridge Gazebo RGB/depth camera topics into ROS 2, estimate sparse optical-flow velocity using per-feature depth scaling, and fuse that velocity with PX4 odometry in a 6-state EKF. This is visual odometry and trust-aware sensor fusion, not SLAM or mapping.
Phase 4 adds a narrow Nav2-facing interface through `twinguard_swarm_nav2_cpp`: a BT plugin that gates Nav2 behavior on TwinGuard trust, and a costmap layer that turns low localization integrity into local planning conservatism. Nav2's global planners, local controllers, obstacle layers, and recovery behaviors remain Nav2's own implementations.
Phase 5 adds deployment realism through `docker/ros2_ws`, `docker/px4_sitl`, and `docker-compose.microservices.yaml`. The microservice profile uses a Fast-DDS discovery server on a Docker bridge network instead of relying on multicast discovery across Docker's default networking. Container boundaries follow subsystem coupling: simulation, autonomy core, Nav2, logging, and discovery.

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

The real dataset is used as a replayed degradation profile for sensor/integrity validation. It does not replace the PX4 flight controller or directly define the UAV path. The C++ `formation_supervisor_node` owns both the circular mission trajectory and the integrity-aware command response.

## Three-UAV Formation Validation Run

The next validation target is a lightweight swarm run with three PX4 `x500` vehicles:

```text
Drone 0: nominal leader-style mission controller
Drone 1: nominal formation follower
Drone 2: dataset-degraded UAV with TwinGuard replay/integrity scoring
```

The formation uses three synchronized C++ formation supervisors with fixed spatial offsets. Drone 2 receives the real dataset replay on its integrity path so the video shows a visible swarm mission while the diagnostics show degradation, residual growth, and trust response for the affected vehicle.

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

This repository defines the ROS 2 package structure, autonomy-layer interfaces, topic contract, setup plan, C++ integrity-scoring package, C++ trust-gated formation supervisor, BehaviorTree.CPP mission selection, local A* rerouting, EKF/visual-odometry estimation path, Nav2-facing trust plugins, deployment container scaffolding, and real dataset replay bridge. The current default path closes the loop from PX4 odometry to C++ trust scoring to BT-guided setpoint selection to C++ offboard command publication. The opt-in Phase 3 launch swaps in `ekf_integrity_node`, which fuses PX4 position with sparse optical-flow visual-odometry velocity while preserving the same `trust_state` interface. The Phase 4 Nav2 package exposes that same trust contract to Nav2 without replacing Nav2 planners. Phase 5 provides reproducible Docker artifacts and a Fast-DDS microservice deployment profile, but full `colcon`, SITL, plugin-load, and Jetson validation still need to run on an Ubuntu ROS/Nav2 machine. Trajectory-risk monitoring is stubbed as a no-op `TrajectoryClear` BT condition until a shared multi-agent trajectory-intent topic is added.

## Intended Outcome

The intended final artifact is a reproducible UAV swarm simulation pipeline where dashboard values, metrics, and command-center videos are generated from PX4/Gazebo/ROS 2 experiment logs.
