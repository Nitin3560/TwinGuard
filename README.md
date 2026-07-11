# TwinGuard

> **Autonomy assurance for UAV swarms — trust-gated control, behavior-tree supervision, and Nav2 integration built on ROS 2, PX4 SITL, and Gazebo.**

![ROS 2 Jazzy](https://img.shields.io/badge/ROS_2-Jazzy-22314E?style=flat&logo=ros&logoColor=white)
![PX4 SITL](https://img.shields.io/badge/PX4-SITL-7B2D8B?style=flat)
![Gazebo Harmonic](https://img.shields.io/badge/Gazebo-Harmonic-F58113?style=flat)
![BehaviorTree.CPP](https://img.shields.io/badge/BehaviorTree.CPP-v3-4A90D9?style=flat)
![Nav2](https://img.shields.io/badge/Nav2-Plugins-2E86AB?style=flat)
![EKF](https://img.shields.io/badge/6--State-EKF-success?style=flat)
![Visual Odometry](https://img.shields.io/badge/Sparse-Visual_Odometry-success?style=flat)
![Digital Twin](https://img.shields.io/badge/Digital-Twin-success?style=flat)
![Trust Scoring](https://img.shields.io/badge/Continuous-Trust_Scoring-success?style=flat)
![Dataset Replay](https://img.shields.io/badge/Dataset-Replay-success?style=flat)

---

TwinGuard is a ROS 2 and PX4-based autonomy framework for making UAV swarms more resilient when localization becomes unreliable.

Most autonomy systems assume localization is trustworthy. When GPS is spoofed, communication degrades, or sensor measurements become inconsistent, planners and controllers often continue operating on corrupted state estimates or immediately fall back to an emergency failsafe.

TwinGuard is built around a different idea: **integrity is a continuous signal, not a binary flag.**

Instead of deciding whether a UAV has "failed," TwinGuard continuously estimates how trustworthy each vehicle's state is and shares that information across planning, supervision, and control. A degraded UAV slows down, reroutes, or holds position proportionally to its confidence instead of failing catastrophically.

The goal is simple: **make localization integrity part of the autonomy pipeline rather than an isolated fault detector.**

---

## System Overview

TwinGuard is organized around one simple idea: **estimation, planning, and control remain independent while sharing a common trust interface.**

The integrity node estimates trust from PX4 odometry. The Behavior Tree selects mission intent, and the Offboard Supervisor applies the final authority gate before commands reach PX4. Nav2 consumes the same trust information without modifying its planners or controllers.

An optional EKF integrity node can replace the default estimator because both publish the same `trust_state` interface.

---

## Architecture

<p align="center">
  <img src="docs/architecture.png" width="780" alt="TwinGuard Architecture"/>
</p>

The architecture separates estimation, planning, and control into independent components connected through a common trust interface. This makes it possible to replace the integrity estimator, extend planning behavior, or integrate Nav2 without changing the rest of the autonomy pipeline.

---

## Engineering Decisions

**Continuous trust instead of binary fault detection.**
Residuals feed a continuously varying trust score. Mild degradation slows the drone down. Severe degradation triggers a hold or reroute. The response is proportional to the confidence in the data, not a threshold crossing.

**Mission planning and safety remain independent.**
The Behavior Tree decides what the UAV should try to do. The Offboard Supervisor decides whether that command actually reaches PX4. Those responsibilities live in separate code and cannot override each other — mission logic cannot bypass the safety gate.

**One trust interface across the entire stack.**
Every component — the supervisor, the BT leaves, both Nav2 plugins — reads from the same `trust_state` topic: trust score, residual, authority scale. Swapping the default integrity node for the EKF-fused version changes nothing downstream.

**Nav2 integration is additive.**
TwinGuard extends Nav2 rather than replacing it. Localization confidence is exposed through a Behavior Tree condition and a custom costmap layer while existing planners, controllers, and recovery behaviors remain unchanged.

---
## Packages

| Package | Language | Responsibility |
|---|---|---|
| `twinguard_swarm_integrity_cpp` | C++ | Lightweight Digital twin prediction, integrity scoring, trust management, formation supervision, authority-gated offboard control |
| `twinguard_swarm_planning_cpp` | C++ | BehaviorTree.CPP mission supervision, obstacle checking, local 3D A* planner |
| `twinguard_swarm_estimation_cpp` | C++ | Sparse optical-flow visual odometry, 6-state EKF, EKF integrity pipeline |
| `twinguard_swarm_nav2_cpp` | C++ | Nav2 BT condition plugin and localization-aware costmap layer |
| `twinguard_dataset_replay` | Python | Dataset-driven degradation replay into live PX4 odometry |
| `twinguard_swarm_bringup` | Python | Launch configurations for integrity, replay, EKF, single-UAV, and multi-UAV experiments |

---
## Status

| Component | Status |
|---|:---:|
| C++ integrity scoring + trust manager | ✅ |
| Trust-gated offboard supervisor | ✅ |
| BehaviorTree.CPP mission supervision | ✅ |
| Local 3D A* rerouting | ✅ |
| Real dataset replay validation | ✅ |
| Sparse optical-flow visual odometry | ✅ |
| 6-state EKF integrity pipeline | ✅ |
| Nav2 BT condition plugin | ✅ |
| Nav2 localization-aware costmap layer | ✅ |
| Docker microservice deployment | ✅ |
| Fast DDS Discovery Server | ✅ |
| Full `colcon` build + Ubuntu/Jazzy validation | ⏳ Pending |
| Multi-agent trajectory conflict monitoring | ⏳ Planned |

---

## Simulation Fidelity

TwinGuard is validated in Gazebo using both live PX4 SITL and replayed localization degradation.

The companion project **sim-val** measures the sensing fidelity gap between Gazebo and NVIDIA Isaac Sim RTX and evaluates how simulator fidelity affects TwinGuard's trust estimates. The resulting analysis is used to guide integrity threshold calibration.

---

## Docs

[Architecture](docs/architecture.md) · [Topic Contract](docs/topic_contract.md) · [Quickstart](docs/quickstart.md) · [Deployment](docs/deployment.md) · [Dataset Replay](docs/real_dataset_replay.md)

---

> **TwinGuard treats localization integrity as a shared system signal rather than a standalone fault detector, allowing estimation, planning, and control to adapt together as confidence changes.**
