# TwinGuard

> **Autonomy assurance for UAV swarms — trust-gated control, behavior-tree supervision, and Nav2 integration built on ROS 2, PX4 SITL, and Gazebo.**

---

Most autonomous UAV systems assume their localization is trustworthy. When GPS is spoofed, communication quality drops, or sensor measurements become unreliable, the planner and controller continue operating on corrupted state estimates. The result is often unstable behavior, poor decisions, or complete mission failure.

**TwinGuard** was built to make **navigation integrity a first-class signal** throughout the autonomy stack.

Instead of treating failures as a binary event, TwinGuard continuously estimates how trustworthy each UAV's state is, converts that estimate into a smooth authority score, and shares it with every major autonomy component. Planning, supervision, and control all make decisions using the same trust information, allowing the vehicle to slow down, reroute, or hold position before localization errors become dangerous.

The goal is not simply to detect faults—it is to keep autonomous systems operating safely while localization quality changes over time.

---

# How It Works

Every UAV runs the following pipeline continuously:

1. **PX4 publishes live vehicle odometry**
2. A lightweight **digital twin predictor** estimates the expected vehicle state.
3. The measured state is compared against that prediction to compute a residual.
4. The residual is converted into a continuous **trust score** and **authority scale**.
5. The **Behavior Tree** selects the most appropriate mission objective.
6. The **Offboard Supervisor** always performs the final safety check before publishing commands to PX4.
7. The same trust information is simultaneously available to **Nav2**, allowing navigation behavior to adapt without modifying the Nav2 stack itself.

Unlike many fault-detection systems, TwinGuard does not immediately switch between "healthy" and "failed." Instead, trust degrades gradually, allowing the vehicle to respond proportionally whenever possible.

---

# Architecture

> *(Replace this section with your final architecture diagram once exported.)*

```text
                 PX4 SITL + Gazebo
                         │
                         ▼
                ROS 2 / PX4 Bridge
                         │
        ┌─────────────────────────────────────┐
        │                                     │
        │        TwinGuard Core               │
        │                                     │
        │ integrity_node_cpp                  │
        │  ├── DigitalTwinPredictor           │
        │  ├── Residual Computation           │
        │  └── TrustScorer                    │
        │             │                       │
        │             ▼                       │
        │         trust_state                 │
        │                                     │
        │ formation_supervisor_node           │
        │  ├── BehaviorTree.CPP               │
        │  │     attack_hold                  │
        │  │     A* reroute                   │
        │  │     nominal mission              │
        │  │                                  │
        │  └── OffboardSupervisor             │
        │        Authority Gate               │
        └─────────────┬───────────────┬───────┘
                      │               │
                      ▼               ▼
             PX4 Offboard        Nav2 Plugins
               Commands
```

The architecture intentionally separates **decision making** from **command execution**.

The Behavior Tree decides **what the UAV should try to do**, while the Offboard Supervisor decides **whether that command should actually be allowed based on current integrity**.

This keeps mission logic independent from safety logic and ensures that authority limiting is always the final step before PX4 receives a command.

---

## Trust Pipeline

TwinGuard publishes one shared message that represents the health of the vehicle.

```text
geometry_msgs/PointStamped  (trust_state)

point.x  → trust score
point.y  → residual
point.z  → authority scale
```

Every major subsystem reads the exact same topic:

- Offboard Supervisor
- Behavior Tree
- Nav2 Behavior Tree plugin
- Nav2 Costmap Layer

Because every component consumes the same interface, new autonomy modules can subscribe to trust information without changing the integrity pipeline itself.

---

## Optional EKF Integrity Pipeline

The default integrity node uses a lightweight constant-velocity digital twin to generate prediction residuals.

For higher-fidelity estimation, TwinGuard also includes an optional EKF pipeline that fuses:

- PX4 position odometry
- Sparse optical-flow visual odometry
- Depth-scaled motion estimates

The visual odometry implementation tracks image features using sparse optical flow and estimates metric velocity using the corresponding depth image. Instead of accepting or rejecting measurements outright, visual odometry quality directly influences EKF measurement noise, reducing the influence of unreliable updates while still allowing them to contribute when appropriate.

Most importantly, the EKF publishes the exact same `trust_state` message as the default integrity node, meaning the remainder of the autonomy stack requires **no changes** when switching estimators.

---

# Packages

| Package | Language | Responsibility |
|----------|----------|----------------|
| `twinguard_swarm_integrity_cpp` | C++ | Digital twin prediction, integrity scoring, trust management, formation supervision, authority-gated offboard control |
| `twinguard_swarm_planning_cpp` | C++ | BehaviorTree.CPP mission supervision, obstacle checking, local 3D A* planner |
| `twinguard_swarm_estimation_cpp` | C++ | Sparse optical-flow visual odometry, 6-state EKF, EKF integrity node |
| `twinguard_swarm_nav2_cpp` | C++ | Nav2 BT condition plugin and localization-aware costmap layer |
| `twinguard_dataset_replay` | Python | Dataset-driven degradation injection into live PX4 odometry |
| `twinguard_swarm_bringup` | Python | Launch files for integrity, replay, EKF, single-UAV, and multi-UAV experiments |

---

# Engineering Decisions Worth Noting

### Continuous trust instead of binary fault detection

TwinGuard intentionally avoids a simple healthy/failed decision.

Residuals are converted into a continuously varying trust score that gradually reduces vehicle authority as localization quality deteriorates. Mild degradation results in slower motion, while severe degradation can trigger a hold or reroute depending on the mission context.

---

### The Behavior Tree never controls PX4 directly

The Behavior Tree only decides **which mission objective should be followed**.

It may choose:

- Hold Position
- Local A* Reroute
- Nominal Mission

Those decisions are forwarded to the Offboard Supervisor, which always performs the final authority scaling before publishing commands to PX4.

Separating mission logic from safety logic keeps responsibilities clear and prevents mission code from bypassing integrity constraints.

---

### Lightweight digital twin by design

The default digital twin is intentionally simple.

Rather than using a computationally expensive physics model, it propagates vehicle state using a constant-velocity prediction that is continuously corrected using incoming PX4 odometry. This produces stable prediction residuals while remaining suitable for real-time onboard execution.

---

### One trust interface for the entire autonomy stack

Every major subsystem consumes the exact same trust message.

Changing the integrity implementation—from the default predictor to the EKF estimator—does not require modifications anywhere else in the autonomy stack because the published interface remains identical.

---

### Nav2 integration is additive

TwinGuard does not replace Nav2.

Instead, it extends it.

The custom Nav2 components expose localization confidence through:

- a Behavior Tree condition node
- a localization-aware costmap layer

Unlike traditional costmaps that represent external obstacles, TwinGuard's costmap layer represents uncertainty around the robot's own estimated position. Existing planners, controllers, and recovery behaviors remain unchanged while automatically benefiting from integrity information.

---

### Dataset-driven validation

Instead of generating artificial faults, TwinGuard can replay real degradation profiles collected from datasets.

Dataset error, quality, and anomaly information are converted into controlled perturbations that are injected into live PX4 odometry before entering the integrity pipeline. This makes it possible to evaluate trust behavior under realistic degradation patterns while preserving the remainder of the autonomy stack unchanged.

---

### Microservice-oriented deployment

TwinGuard separates the autonomy stack into independent services.

- PX4 SITL
- ROS 2 autonomy core
- Nav2
- experiment logging

Fast DDS Discovery Server replaces multicast discovery, allowing the services to communicate reliably across Docker bridge networks while keeping safety-critical autonomy isolated from optional components such as logging.

The intended final artifact is a reproducible UAV swarm simulation pipeline where dashboard values, metrics, and command-center videos are generated from PX4/Gazebo/ROS 2 experiment logs.
