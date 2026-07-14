# TwinGuard Architecture

TwinGuard is a trust-aware autonomy framework built on ROS 2, PX4 SITL, and Gazebo for improving the resilience of UAV autonomy when localization becomes unreliable.

Rather than treating localization integrity as an isolated fault-detection problem, TwinGuard integrates localization confidence directly into the autonomy pipeline. Every major subsystem—including planning, supervision, navigation, and control—adapts its behavior using a shared trust estimate derived from live vehicle state.

The architecture was designed around one central question:

> **How should an autonomous UAV behave when it no longer completely trusts its own localization?**

Instead of responding to degraded localization with a binary failsafe, TwinGuard continuously estimates localization integrity and proportionally adjusts vehicle behavior. This allows autonomy to degrade gracefully rather than abruptly terminating a mission whenever localization quality decreases.

---

# Architecture Overview

![TwinGuard Architecture](architecture.png)

TwinGuard separates estimation, planning, supervision, and control into independent components connected through a shared runtime interface.

Rather than allowing each subsystem to estimate localization quality independently, the integrity pipeline computes trust once and publishes a common `trust_state` interface that every downstream component consumes.

This architecture provides three important benefits:

- Every subsystem makes decisions using the same integrity estimate.
- Individual components remain loosely coupled and independently replaceable.
- New estimators or planners can be introduced without modifying the rest of the autonomy stack.

---

# Design Principles

Several principles guided the overall architecture.

### Estimate integrity once

Localization confidence is estimated inside a dedicated integrity pipeline rather than separately by planners, controllers, or navigation modules. This prevents inconsistent confidence estimates across the system.

### Publish a shared trust interface

Every downstream component consumes the same `trust_state` message instead of computing its own integrity estimate.

### Separate mission planning from safety

Mission logic determines what the vehicle should attempt to do, while a dedicated supervisor determines whether those commands are safe to execute.

### Keep subsystems loosely coupled

Each subsystem performs one responsibility while communicating only through well-defined ROS 2 interfaces.

### Extend existing autonomy software

TwinGuard extends PX4, Nav2, and ROS 2 rather than replacing their existing capabilities.

---

# Runtime Overview

Each control cycle follows the same execution sequence.

1. PX4 SITL and Gazebo generate the simulated vehicle state.
2. The ROS 2 bridge exposes PX4 telemetry to the autonomy runtime.
3. TwinGuard evaluates localization integrity.
4. Mission planning and navigation consume the shared trust estimate.
5. The Offboard Supervisor applies the final authority gate.
6. Authority-scaled commands are transmitted back to PX4.

Every iteration begins with new vehicle state arriving from PX4.

The runtime does not form a feedback loop from planner outputs back into the integrity estimator. Every control cycle starts with fresh vehicle observations, ensuring that localization integrity is always evaluated using current sensor information rather than previous control decisions.

---

# Integrity Estimation Pipeline

The integrity pipeline forms the foundation of the TwinGuard architecture.

Its responsibility is to determine how trustworthy the current localization estimate is before any planning or control decisions are made.

The pipeline receives live PX4 vehicle odometry and compares it against a lightweight model-based state prediction.

The difference between predicted and measured vehicle state forms a residual that is continuously evaluated over time.

Rather than generating a binary fault decision, TwinGuard converts this residual into two runtime quantities:

- Trust score
- Authority scale

The trust score represents confidence in the current localization estimate.

The authority scale represents how much control authority should be granted to downstream autonomy components.

These values are continuously updated throughout flight, allowing gradual adaptation instead of abrupt transitions between normal operation and emergency failsafe behavior.

---

# Model-Based State Prediction

TwinGuard intentionally uses a lightweight constant-velocity prediction model instead of a computationally expensive vehicle dynamics simulation.

The objective of the predictor is not to perfectly model UAV dynamics.

Its purpose is to provide a stable reference against which incoming localization measurements can be evaluated.

A lightweight prediction model offers several advantages:

- deterministic execution,
- predictable runtime,
- minimal computational overhead,
- stable residual generation suitable for continuous trust estimation.

This makes it significantly more practical for real-time integrity estimation than a complex physics model while still providing sufficient information for residual analysis.

---

# Shared Trust Interface

One of the central architectural abstractions in TwinGuard is the shared runtime interface:

```text
trust_state
```

Rather than publishing multiple integrity messages for different subsystems, TwinGuard publishes a single message that represents the current localization confidence.

The message contains information including:

- localization trust,
- residual magnitude,
- authority scaling,
- fault classification.

Every major subsystem consumes this interface.

This includes:

- BehaviorTree mission supervision,
- Offboard Supervisor,
- Nav2 plugins,
- formation supervision,
- optional integrity estimators.

Because every subsystem relies on the same runtime interface, localization confidence remains consistent throughout the autonomy stack.

The integrity estimator can therefore evolve independently without requiring modifications to planners, controllers, navigation components, or supervisory logic.

This separation between estimation and consumption became one of the defining architectural decisions within TwinGuard.

# Mission Planning and Safety

A fundamental architectural decision in TwinGuard is the separation of mission planning from safety enforcement.

Mission planning determines **what the UAV should attempt to do**, while safety enforcement determines **whether those commands should actually be executed**.

Although combining these responsibilities inside a single node would simplify implementation, it would also allow mission logic to bypass localization integrity constraints.

TwinGuard avoids this by separating responsibilities into independent runtime components.

The BehaviorTree.CPP mission supervisor is responsible for high-level mission execution, waypoint sequencing, task selection, and recovery logic.

The Offboard Supervisor remains responsible for evaluating the current authority scale before any PX4 command is transmitted.

Every offboard command passes through the supervisor regardless of its origin.

This separation ensures that mission execution cannot override integrity constraints while allowing both systems to evolve independently.

---

# Formation Supervisor

The Formation Supervisor acts as the final decision layer before PX4 receives control commands.

Rather than estimating localization quality itself, it consumes the published `trust_state` interface together with mission intent produced by the Behavior Tree.

Its responsibilities include:

- BehaviorTree execution
- Local A* replanning
- Authority scaling
- PX4 offboard command publication
- Formation supervision

By combining these responsibilities inside a single supervisory layer, TwinGuard maintains a clear separation between estimation and execution.

Localization confidence influences mission execution without requiring planners or controllers to implement integrity estimation themselves.

---

# Kalman-Based Integrity Estimation

TwinGuard includes an optional Kalman-based integrity pipeline that extends the default residual-based estimator without changing the overall system architecture.

The estimator fuses multiple sensing modalities including:

- PX4 vehicle odometry
- Sparse optical-flow visual odometry
- Depth-scaled motion estimation

Rather than replacing the existing integrity pipeline, the Kalman estimator publishes exactly the same `trust_state` interface.

Every downstream component therefore continues operating without modification.

One important design decision concerns visual odometry quality.

Instead of accepting or rejecting visual measurements using a fixed threshold, TwinGuard adjusts measurement uncertainty according to tracking quality.

When feature tracking deteriorates, visual measurements contribute less to the state estimate instead of being discarded entirely.

This produces smoother transitions while maintaining estimator stability during periods of degraded visual information.

Because both integrity estimators expose the same runtime interface, switching between them requires only launch-time configuration rather than architectural changes.

---

# Nav2 Integration

TwinGuard is designed to complement existing autonomy software rather than replace it.

Nav2 already provides mature planning, control, recovery, and navigation capabilities.

Instead of introducing another navigation framework, TwinGuard extends Nav2 by exposing localization confidence as an additional runtime input.

Two integrations are provided.

### Behavior Tree Condition

A custom Nav2 Behavior Tree condition allows navigation logic to react directly to localization confidence.

Mission execution can therefore adapt automatically when localization integrity decreases.

### Localization-Aware Costmap Layer

TwinGuard also provides a custom costmap layer representing localization uncertainty rather than physical obstacles.

As confidence decreases, navigation behavior becomes progressively more conservative without modifying Nav2 planners or controllers.

This allows existing navigation algorithms to remain unchanged while still responding to degraded localization conditions.

TwinGuard therefore extends Nav2 instead of competing with it.

---

# Dataset Replay

Simulation provides a controlled environment for testing autonomy systems, but realistic localization degradation is difficult to reproduce using synthetic failures alone.

TwinGuard addresses this through dataset replay.

Instead of replaying complete vehicle trajectories, controlled localization perturbations derived from real datasets are injected directly into live PX4 odometry.

From the perspective of the integrity pipeline, the replayed data is indistinguishable from live sensor information.

The remainder of the autonomy stack continues operating normally.

This approach provides:

- repeatable experiments,
- controlled degradation scenarios,
- realistic localization disturbances,
- identical runtime behavior between replay and live simulation.

Because replay occurs only at the localization interface, the overall system architecture remains unchanged.

---

# Modular Deployment

As TwinGuard expanded, separating responsibilities into independent runtime services became increasingly important.

Rather than executing the entire autonomy stack inside a single ROS environment, the system is deployed as several loosely coupled services.

Typical deployment consists of:

- PX4 SITL
- Gazebo simulation
- ROS 2 autonomy runtime
- Nav2
- Dataset replay
- Experiment logging

Communication between services is coordinated through a Fast DDS Discovery Server.

This architecture improves maintainability by allowing individual components to be restarted, updated, or replaced independently while minimizing coupling between runtime services.

It also mirrors deployment strategies commonly used in larger robotics software systems.

---

# Extending the Architecture

One of the primary goals during development was minimizing architectural changes when introducing new functionality.

Because subsystems communicate exclusively through well-defined ROS 2 interfaces, future extensions generally require adding new components rather than modifying existing ones.

Examples include:

- replacing the integrity estimator,
- introducing additional planners,
- integrating alternative localization sources,
- extending Nav2 functionality,
- supporting additional UAV platforms.

As long as new components consume or publish the existing runtime interfaces, they integrate naturally into the existing architecture.

This significantly improves scalability while reducing long-term maintenance complexity.

---

# Architectural Summary

TwinGuard is organized around a simple principle:

> **Estimate localization integrity once, then allow every autonomy component to use that information consistently.**

The integrity pipeline estimates localization confidence.

The Behavior Tree determines mission intent.

The Offboard Supervisor enforces authority constraints.

Nav2 adapts navigation behavior.

Each subsystem performs one well-defined responsibility while remaining loosely coupled from the rest of the autonomy stack.

This separation allows estimation, planning, supervision, and navigation to evolve independently while maintaining a consistent understanding of localization integrity throughout the system.

---

# Closing Remarks

TwinGuard demonstrates that resilient autonomy depends not only on detecting degraded localization, but also on making appropriate decisions after degradation has been identified.

Rather than treating integrity as an isolated fault-detection problem, the architecture promotes localization confidence to a shared runtime signal consumed throughout the autonomy stack.

By separating estimation, planning, supervision, and control while connecting them through a common trust interface, TwinGuard provides an architecture that is modular, extensible, and significantly easier to validate than tightly coupled autonomy systems.

This design philosophy guides every subsystem within TwinGuard and provides a foundation for future extensions involving additional estimators, planners, navigation frameworks, and multi-UAV coordination.
