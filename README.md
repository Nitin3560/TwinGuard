what do you think abou tthis "# TwinGuard

> **Autonomy assurance for UAV swarms — trust-gated control, behavior-tree supervision, and Nav2 integration built on ROS 2, PX4 SITL, and Gazebo.**

![ROS 2](https://img.shields.io/badge/ROS_2-Jazzy-22314E?style=flat&logo=ros&logoColor=white)
![C++](https://img.shields.io/badge/C++-17-00599C?style=flat&logo=cplusplus&logoColor=white)
![PX4](https://img.shields.io/badge/PX4-SITL-7B2D8B?style=flat)
![Gazebo](https://img.shields.io/badge/Gazebo-Harmonic-F58113?style=flat)
![Nav2](https://img.shields.io/badge/Nav2-integrated-2E86AB?style=flat)
![BehaviorTree](https://img.shields.io/badge/BehaviorTree.CPP-v3-4A90D9?style=flat)

---

TwinGuard is a ROS 2 and PX4-based autonomy framework for making UAV swarms more resilient when localization becomes unreliable.

Most autonomy systems assume localization is trustworthy. When GPS is spoofed, communication degrades, or sensor measurements become inconsistent, planners and controllers often continue operating on corrupted state estimates or immediately fall back to an emergency failsafe.

TwinGuard explores a different idea: **integrity is a continuous signal, not a binary flag.**

Instead of deciding whether a UAV has "failed," TwinGuard continuously estimates how trustworthy each vehicle's state is and shares that information across planning, supervision, and control. A degraded UAV slows down, reroutes, or holds position proportionally to its confidence instead of failing catastrophically.

The goal is simple: **make localization integrity part of the autonomy pipeline rather than an isolated fault detector.**

---

## How It Works

TwinGuard is built around one idea:

**Estimation, planning, and control should remain independent.**

The integrity node estimates trust. The Behavior Tree decides what the UAV should try to do. The Offboard Supervisor decides whether that command should actually reach PX4.

Keeping those responsibilities separate makes the system easier to reason about, easier to extend, and much harder for mission logic to bypass safety constraints.

For every control cycle:

- PX4 publishes the latest vehicle odometry
- A lightweight digital twin predicts the expected vehicle state
- The measured state is compared against that prediction to compute a residual
- The residual is converted into a continuous trust score
- Trust is mapped into an authority scale
- The Behavior Tree selects the most appropriate mission objective
- The Offboard Supervisor applies the final authority gate before publishing commands to PX4
- Nav2 consumes the same trust information without requiring any changes to its planners or controllers

---

## Architecture

<p align="center">
  <img src="docs/images/architecture.png" width="780" alt="TwinGuard Architecture"/>
</p>

The architecture intentionally separates **decision making** from **command execution**. The Behavior Tree determines what mission objective should be followed. The Offboard Supervisor determines whether that command should actually be sent to PX4 based on the current integrity estimate. Safety enforcement remains independent of mission logic.

---

## Core Packages

| Package | Language | Responsibility |
|---|---|---|
| `twinguard_swarm_integrity_cpp` | C++ | Digital twin prediction, integrity scoring, trust management, formation supervision, authority-gated offboard control |
| `twinguard_swarm_planning_cpp` | C++ | BehaviorTree.CPP mission supervision, obstacle checking, local 3D A* planner |
| `twinguard_swarm_estimation_cpp` | C++ | Sparse optical-flow visual odometry, 6-state EKF, EKF integrity pipeline |
| `twinguard_swarm_nav2_cpp` | C++ | Nav2 BT condition plugin and localization-aware costmap layer |
| `twinguard_dataset_replay` | Python | Dataset-driven degradation replay into live PX4 odometry |
| `twinguard_swarm_bringup` | Python | Launch configurations for integrity, replay, EKF, single-UAV, and multi-UAV experiments |

---

## Engineering Decisions

**Continuous trust instead of binary fault detection.**
Residuals feed a continuously varying trust score. Mild degradation slows the drone down. Severe degradation triggers a hold or reroute. The response is proportional to the confidence in the data, not a threshold crossing.

**Mission planning and safety remain independent.**
The Behavior Tree decides what the UAV should try to do. The Offboard Supervisor decides whether that command actually reaches PX4. Those responsibilities live in separate code and cannot override each other — mission logic cannot bypass the safety gate.

**One trust interface across the entire stack.**
Every component — the supervisor, the BT leaves, both Nav2 plugins — reads from the same `trust_state` topic: trust score, residual, authority scale. Swapping the default integrity node for the EKF-fused version changes nothing downstream.

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

TwinGuard's integrity thresholds are currently calibrated using Gazebo. The companion project [sim-val](https://github.com/Nitin3560/sim-val) measures the sensing fidelity gap between Gazebo ray-casting and NVIDIA Isaac Sim RTX under identical scenarios, then evaluates how that difference propagates into TwinGuard's EKF trust estimates. The long-term goal is simulator-specific integrity calibration instead of manually tuned thresholds.

---

## Docs

[Architecture](docs/architecture.md) · [Topic Contract](docs/topic_contract.md) · [Quickstart](docs/quickstart.md) · [Deployment](docs/deployment.md) · [Dataset Replay](docs/real_dataset_replay.md)

---

> **TwinGuard demonstrates how localization integrity can become a shared decision-making signal across an entire autonomy stack, allowing UAVs to respond proportionally to degraded sensing instead of treating every fault as an immediate failure.**"
