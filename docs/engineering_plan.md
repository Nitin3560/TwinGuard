# Engineering Decisions

This document is different from the architecture documentation.

The architecture explains how TwinGuard is organized.

This document explains **why I built it that way**, the trade-offs I made, and the engineering problems I encountered while developing the system.

Many of these decisions changed as the project evolved. Looking back, I think these choices had a much bigger impact on the project than any individual algorithm.

---

# 1. I Wanted Integrity To Become A Shared Signal

The first idea I had was actually very simple.

Instead of asking:

> "Has the UAV failed?"

I wanted to ask:

> "How much should the autonomy stack trust the current vehicle state?"

That small change completely influenced the architecture.

If trust became a continuous quantity instead of a binary decision, every subsystem could react differently.

Planning could become more conservative.

Control could reduce authority.

Navigation could avoid risky maneuvers.

Instead of building separate integrity logic into every package, I decided to compute trust once and let every subsystem consume the same estimate.

Looking back, I think this became the most important architectural decision in TwinGuard.

---

# 2. Why I Didn't Build A Complex Digital Twin

One obvious direction would have been creating a sophisticated vehicle dynamics model.

I deliberately avoided that.

The objective of the digital twin was never perfect prediction.

Its job was to generate stable residuals.

A lightweight constant-velocity predictor is deterministic, computationally inexpensive, and predictable during debugging.

That made it a much better engineering choice than introducing additional model complexity that wouldn't significantly improve trust estimation.

Sometimes simpler really is better.

---

# 3. Why I Separated Planning From Safety

This decision came after I started implementing the Behavior Tree.

Initially, I considered allowing the planner to publish PX4 commands directly.

The more I worked on the system, the more uncomfortable I became with that idea.

If planning and safety live inside the same component, it becomes very easy for future changes to accidentally bypass integrity constraints.

Instead, I split the responsibilities.

The Behavior Tree decides what the UAV wants to do.

The Offboard Supervisor decides whether the UAV is actually allowed to do it.

Every command passes through the supervisor before reaching PX4.

That separation made the entire control pipeline much easier to reason about.

---

# 4. Why Everything Uses trust_state

Early versions of the project had multiple pieces of integrity information flowing between packages.

That quickly became difficult to maintain.

Different packages started needing different values.

Instead of creating more interfaces, I replaced them with one shared contract.

```
trust_state
```

Every subsystem now consumes exactly the same information.

The planner.

The supervisor.

Nav2.

Future packages.

That decision dramatically reduced coupling across the project and made replacing the integrity estimator much easier later.

---

# 5. Why I Didn't Modify Nav2

Nav2 already solves navigation extremely well.

I didn't want TwinGuard to become another navigation framework.

Instead, I asked myself:

> "Can integrity simply become another input to Nav2?"

That led to two plugins.

A Behavior Tree condition.

A localization-aware costmap layer.

Nav2 continues doing what it already does well while automatically considering localization confidence.

This allowed me to extend Nav2 rather than fork it.

---

# 6. Why I Built Dataset Replay

Simulation alone wasn't enough.

Randomly injecting Gaussian noise also wasn't enough.

I wanted repeatable experiments using realistic degradation.

Instead of replaying entire trajectories, I inject controlled localization degradation into live PX4 odometry.

The autonomy stack continues operating normally.

Only the localization quality changes.

That made the experiments both repeatable and representative of real degradation.

---

# 7. Why I Used Docker

As the project grew, running everything inside one ROS workspace became increasingly difficult.

Different components had different responsibilities.

PX4.

Autonomy.

Navigation.

Logging.

They didn't all need to restart together.

Separating them into containers made debugging much easier and naturally pushed the architecture toward cleaner interfaces.

Using a Fast DDS Discovery Server also removed many of the networking issues that appear when ROS 2 relies on multicast inside Docker.

---

# Engineering Challenges

Most of the difficult problems I encountered weren't algorithmic.

They were architectural.

---

## Keeping Components Independent

The biggest challenge was making sure estimation, planning, supervision, and navigation remained independent.

Every time two components started depending on each other, I tried to move that information into a shared interface instead.

That eventually led to the trust_state message becoming the center of the architecture.

---

## Avoiding Tight Coupling

It was tempting to let one node directly call another or expose implementation details.

I intentionally avoided that.

Every package communicates through ROS topics or well-defined interfaces.

That made replacing components much easier later in development.

---

## Balancing Simplicity And Capability

A recurring decision throughout the project was choosing between adding another sophisticated algorithm or keeping the system understandable.

Whenever possible, I preferred simpler implementations with clear responsibilities over complicated solutions that solved multiple problems at once.

I wanted someone reading the code to understand why every component existed.

---

## Designing For Extension

I knew the project would continue growing.

Because of that, I tried to design every major component so it could be replaced independently.

Today the integrity estimator can be swapped.

New planners can be added.

Nav2 can evolve independently.

That flexibility came from spending time on interfaces early instead of adding them later.

---

# Looking Back

The biggest lesson I learned while building TwinGuard is that resilient autonomy is much more about architecture than algorithms.

Digital twins, EKFs, planners, and controllers are all important.

But if they cannot communicate through simple, well-defined interfaces, the system quickly becomes difficult to extend and difficult to trust.

If I started the project again today, I would make many implementation improvements.

I would not change the core architectural decision.

Computing trust once and allowing every subsystem to make decisions from the same integrity estimate is still the design choice I am most confident in.
