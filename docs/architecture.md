# TwinGuard Architecture

TwinGuard is organized around one design principle:

> **Estimation, planning, and control remain independent while sharing a common trust interface.**

Instead of allowing each subsystem to estimate localization quality independently, TwinGuard computes integrity once and distributes that information throughout the autonomy stack. Every component reacts to the same trust estimate while remaining loosely coupled from the rest of the system.

This document explains how the runtime is organized, how information flows between components, and why each subsystem exists.

---

# System Overview

![TwinGuard Architecture](architecture.png)

The runtime consists of five primary stages:

1. PX4 simulation and sensing
2. ROS 2 communication bridge
3. TwinGuard runtime
4. PX4 offboard interface
5. Nav2 trust integration

Each stage has a single responsibility and communicates through ROS topics and PX4 messages.

---

# Runtime Pipeline

The runtime executes the following sequence continuously.

```
PX4 VehicleOdometry
        ↓
Integrity Estimation
        ↓
Trust Evaluation
        ↓
Mission Supervision
        ↓
Authority Enforcement
        ↓
PX4 Offboard Interface
```

Every new `VehicleOdometry` message starts a new integrity update. There is no feedback path from the outputs back into the integrity node. The next iteration always begins with fresh state information from PX4.

---

# Runtime Components

## PX4 SITL + Gazebo

PX4 SITL provides the simulated flight controller while Gazebo provides the simulation environment and onboard sensors.

The default configuration uses the x500 model together with RGB and depth cameras.

This stage represents the physical UAV and produces the state information consumed by TwinGuard.

---

## ROS 2 / PX4 Bridge

Micro XRCE-DDS bridges PX4 messages into the ROS 2 ecosystem.

This allows autonomy components to subscribe to PX4 state while publishing offboard commands back to the flight controller.

The bridge intentionally contains no autonomy logic.

Its only responsibility is communication.

---

## TwinGuard Runtime

The TwinGuard runtime contains the integrity estimator, mission supervisor, and trust-aware control logic.

Every component inside the runtime is responsible for one stage of the autonomy pipeline.

---

### integrity_node_cpp

The integrity node receives PX4 vehicle odometry and estimates how trustworthy the current vehicle state is.

The default implementation consists of three parts:

- DigitalTwinPredictor
- Residual computation
- TrustScorer

The Digital Twin predicts the expected vehicle state using a lightweight constant-velocity model.

The measured state is compared against that prediction to generate a residual.

The residual is converted into a continuously varying trust score together with an authority scale that downstream components use to modify vehicle behavior.

The integrity node publishes a single shared interface:

```
trust_state
```

Every other TwinGuard component consumes this message.

---

### Optional EKF Integrity Node

TwinGuard also provides an optional EKF-based integrity estimator.

Instead of relying only on PX4 odometry, the EKF fuses:

- PX4 position estimates
- Sparse optical-flow visual odometry
- Depth-scaled motion estimates

Visual odometry quality directly influences EKF measurement uncertainty, allowing unreliable measurements to contribute proportionally instead of being simply accepted or rejected.

The EKF publishes the exact same `trust_state` interface as the default integrity node.

Because of this common interface, the remainder of the autonomy stack does not change when switching estimators.

---

### trust_state

`trust_state` is the central interface used throughout TwinGuard.

Rather than allowing every subsystem to compute integrity independently, TwinGuard computes trust once and distributes it throughout the system.

The message contains:

- Trust score
- Residual
- Authority scale

This interface is consumed by:

- Formation Supervisor
- Behavior Tree
- Nav2 plugins

Using one shared interface keeps the architecture loosely coupled and allows integrity algorithms to evolve without affecting downstream components.

---

### formation_supervisor_node

The formation supervisor coordinates mission execution.

Its responsibilities include:

- Behavior Tree execution
- Local path replanning
- Authority scaling
- PX4 offboard command generation

The supervisor never estimates integrity itself.

Instead, it consumes `trust_state` and adapts vehicle behavior accordingly.

---

## Behavior Tree

Mission selection is implemented using BehaviorTree.CPP.

The current decision order is:

1. Attack hold
2. Local A* reroute
3. Nominal mission

The Behavior Tree determines **what the UAV should attempt to do**.

It does **not** communicate directly with PX4.

---

## Offboard Supervisor

The Offboard Supervisor represents the final authority gate before commands reach PX4.

Every candidate trajectory generated by the Behavior Tree passes through the supervisor.

Depending on the current authority scale, the supervisor may:

- publish the nominal command
- reduce vehicle authority
- hold position

Keeping the supervisor outside the Behavior Tree separates mission planning from safety enforcement.

---

# PX4 Offboard Interface

TwinGuard communicates with PX4 using the standard offboard interface.

The runtime publishes:

- OffboardControlMode
- TrajectorySetpoint
- VehicleCommand

These messages represent the final output of the autonomy pipeline.

---

# Nav2 Trust Plugins

TwinGuard extends Nav2 without modifying the navigation stack itself.

Two plugins expose localization integrity to Nav2:

### IsAgentTrustworthy

A Behavior Tree condition node that allows navigation logic to react to localization confidence.

### TwinGuardIntegrityLayer

A custom costmap layer that represents uncertainty around the robot's own estimated position.

Unlike traditional costmap layers that represent external obstacles, this layer represents localization confidence.

Existing Nav2 planners and controllers remain unchanged.

---

# Data Flow Summary

The runtime follows one continuous processing loop.

```
VehicleOdometry
      ↓
Digital Twin / EKF
      ↓
Residual
      ↓
Trust
      ↓
Authority
      ↓
Behavior Tree
      ↓
Offboard Supervisor
      ↓
PX4 Commands
```

Every iteration begins with fresh PX4 state information and ends with a new set of offboard commands.

---

# Design Philosophy

TwinGuard intentionally separates estimation, planning, and control into independent components.

Each subsystem has one clearly defined responsibility.

Trust is computed once.

Mission planning consumes trust.

Safety enforcement consumes trust.

Navigation consumes trust.

This separation makes the architecture easier to extend, easier to validate, and allows individual components to evolve without changing the remainder of the autonomy stack.
