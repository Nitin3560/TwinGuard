# TwinGuard

> **Trust-aware autonomy framework for UAVs built on ROS 2, PX4 SITL, and Gazebo. TwinGuard continuously estimates localization integrity and adapts planning, supervision, and offboard control before unreliable state estimates propagate through the autonomy stack.**

![ROS 2 CI](https://github.com/Nitin3560/TwinGuard/actions/workflows/ros2-ci.yml/badge.svg)
![ROS 2 Jazzy](https://img.shields.io/badge/ROS_2-Jazzy-22314E?style=flat&logo=ros&logoColor=white)
![PX4 SITL](https://img.shields.io/badge/PX4-SITL-7B2D8B?style=flat)
![Gazebo Harmonic](https://img.shields.io/badge/Gazebo-Harmonic-F58113?style=flat)
![C++17](https://img.shields.io/badge/C++-17-blue?style=flat)
![GoogleTest](https://img.shields.io/badge/GoogleTest-Passing-success?style=flat)
![BehaviorTree.CPP](https://img.shields.io/badge/BehaviorTree.CPP-v3-4A90D9?style=flat)
![Nav2](https://img.shields.io/badge/Nav2-Plugins-2E86AB?style=flat)

---

## Demo

<p align="center">
<b>End-to-end PX4 SITL demonstration showing nominal operation, localization degradation, trust-aware supervision, and recovery.</b>
</p>


![TwinGuard Demo](docs/demo-small.gif)

---
## Key Capabilities

- Continuous localization trust estimation
- Trust-aware offboard supervision
- BehaviorTree.CPP mission execution
- Nav2 localization-aware planning
- PX4 SITL + Gazebo integration
- Dataset replay for repeatable validation
- Automated CI, unit tests, and ROS 2 integration tests

## Architecture

<p align="center">
<img src="docs/architecture.png" width="820" alt="TwinGuard Architecture"/>
</p>

TwinGuard separates **estimation**, **planning**, and **control** through a common trust interface.
Localization confidence is estimated once and consumed throughout the autonomy pipeline rather than embedding fault handling inside every controller.

---

## Why TwinGuard?

Most UAV autonomy stacks assume localization is always trustworthy.

When GPS is spoofed, communication quality degrades, or state estimation becomes unreliable, planners continue making decisions using corrupted state estimates — or immediately trigger a failsafe.

TwinGuard follows a different philosophy.

Instead of treating integrity as a binary fault, **TwinGuard models localization confidence as a continuous runtime signal.**

Residual-based trust estimation continuously adjusts the amount of authority given to autonomy components, allowing the vehicle to:

- continue normal operation,
- slow down,
- reroute,
- or hold position

depending on confidence rather than a single threshold crossing.

---

## System Pipeline

```text
PX4 VehicleOdometry
          │
          ▼
Model-Based State Prediction
          │
Residual Computation
          │
Continuous Trust Estimation
          │
Trust State
          │
 ┌────────┴─────────┐
 │                   │
 ▼                   ▼
BehaviorTree.CPP    Nav2 Plugins
 │                   │
 └────────┬──────────┘
          ▼
Offboard Supervisor
          │
Authority Scaling
          │
TrajectorySetpoint
          │
          ▼
         PX4
```


---

## Validation

TwinGuard is validated using PX4 SITL and Gazebo by injecting localization degradation into the autonomy pipeline and observing how trust-aware supervision adapts vehicle authority.

### Trust-aware supervisory response

<p align="center">
<img src="docs/images/validation_trust_response.png" width="900" alt="Trust-aware supervisory response"/>
</p>

During nominal operation, trust remains high and the supervisor allows full authority. When localization integrity degrades, trust drops rapidly, authority is reduced to a configurable floor, and the supervisor transitions into **degraded-hold** mode to prevent unsafe control commands.

---

### Residual-driven trust collapse

<p align="center">
<img src="docs/images/validation_residual_response.png" width="900" alt="Residual-driven trust collapse"/>
</p>

Localization residuals remain low during normal operation. When degraded localization is introduced, residuals increase sharply, causing the trust estimator to reduce confidence before corrupted state estimates propagate through planning and control.

---
## Packages

| Package | Responsibility |
|---|---|
| `twinguard_swarm_integrity_cpp` | State prediction, trust estimation, authority scaling, formation supervision, PX4 offboard interface |
| `twinguard_swarm_planning_cpp` | BehaviorTree.CPP mission supervision and local A* planning |
| `twinguard_swarm_estimation_cpp` | Visual odometry, 6-state Kalman filter, integrity estimation |
| `twinguard_swarm_nav2_cpp` | Nav2 Behavior Tree condition and localization-aware costmap |
| `twinguard_dataset_replay` | Dataset-driven localization degradation replay |
| `twinguard_swarm_bringup` | Launch files and experiment orchestration |

---

## Engineering Highlights

• Modular ROS 2 package architecture
• PX4 SITL + Gazebo Harmonic integration
• Continuous trust estimation
• Trust-aware offboard supervision
• BehaviorTree.CPP mission supervision
• Nav2 localization-aware plugins
• Dataset replay for repeatable degradation scenarios
• Kalman-based state estimation
• GitHub Actions continuous integration
• GoogleTest unit testing
• ROS 2 launch integration testing
• Docker deployment support

---

## Build and Test Verification

TwinGuard has been validated using a reproducible ROS 2 workflow.

### Continuous Integration

- ✅ Ubuntu 24.04
- ✅ ROS 2 Jazzy
- ✅ Full workspace build
- ✅ GitHub Actions CI
- ✅ Automated package testing

### Unit Testing

GoogleTest coverage includes:

- TrustScorer
- Kalman Estimator
- A* Planner

### Integration Testing

ROS 2 launch testing validates the integrity-supervisor pipeline:

```text
VehicleOdometry
        ↓
Integrity Node
        ↓
Trust State
        ↓
Offboard Supervisor
        ↓
Authority-scaled Commands
```

---

## Repository Status

| Component | Status |
|---|---|
| Trust estimation | ✅ |
| Offboard supervisor | ✅ |
| BehaviorTree mission supervision | ✅ |
| A* planner | ✅ |
| Dataset replay | ✅ |
| Visual odometry | ✅ |
| Kalman estimator | ✅ |
| Nav2 plugins | ✅ |
| Docker deployment | ✅ |
| GitHub Actions CI | ✅ |
| GoogleTest suite | ✅ |
| ROS 2 integration tests | ✅ |
| End-to-end PX4 SITL demonstration | ✅ |
| Multi-UAV conflict monitoring | 🚧 |

---

## Documentation

- [Architecture](docs/architecture.md)
- [Design DOC](docs/Design_doc.md)
- [Topic Contract](docs/topic-contract.md)
- [Quickstart](docs/quickstart.md)
- [Deployment](docs/deployment.md)

---

## Companion Project

The companion project **sim-val** evaluates the sensing fidelity gap between Gazebo and NVIDIA Isaac Sim and studies how simulator fidelity influences TwinGuard's localization integrity estimation and supervisory behavior.
> **TwinGuard treats localization integrity as a shared runtime signal, allowing estimation, planning, and control to adapt together rather than reacting independently to degraded localization.**
