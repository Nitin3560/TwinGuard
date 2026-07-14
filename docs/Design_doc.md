# Engineering Design Decisions

The architecture documentation explains how TwinGuard is organized.

This document explains why I designed it that way.

When I started building TwinGuard, I expected the difficult part of the project to be implementing the algorithms. I thought most of my time would be spent on estimation, planning, and integrating PX4 with ROS 2.

That wasn't what happened.

The algorithms certainly required work, but the real challenge turned out to be designing an architecture where those algorithms could evolve independently without becoming tightly coupled.

As the project grew, I found myself making far more architectural decisions than algorithmic ones. Most of those decisions were driven by maintainability, extensibility, and simplicity rather than pursuing increasingly sophisticated implementations.

This document captures the reasoning behind those decisions.

---

# Why I Made Integrity a Shared Runtime Signal

The very first architectural decision shaped almost everything that followed.

While studying existing autonomy pipelines, I noticed that localization integrity was usually treated as a separate monitoring problem. Planners trusted localization, controllers trusted localization, and navigation stacks trusted localization. If localization quality degraded, each subsystem either continued operating normally or reacted independently.

That approach never felt right to me.

I wanted every autonomy component to have the same understanding of localization quality.

Instead of asking:

> "Has the UAV failed?"

I wanted every subsystem to continuously answer:

> "How much should I trust the current vehicle state?"

That small change completely altered the architecture.

A binary fault detector immediately limits every downstream component to the same response. Once the fault flag becomes true, every subsystem behaves identically regardless of the severity of the degradation.

Real systems rarely behave that way.

Sometimes the correct response is simply reducing vehicle authority.

Sometimes replanning is enough.

Sometimes holding position is appropriate.

Those decisions become possible only when integrity is represented as a continuously varying quantity rather than a Boolean state.

That realization eventually led me to expose localization integrity as a shared runtime interface instead of a fault detector.

Looking back, I think this became the most important design decision in the entire project.

Once the integrity interface became stable, nearly every future extension naturally fit into the architecture.

---

# Why I Chose a Lightweight Prediction Model

One of the earliest questions I asked myself was how sophisticated the prediction model actually needed to be.

Initially I considered implementing a much richer vehicle dynamics model.

On paper, that seemed like the obvious direction.

The more I thought about it, however, the more I realized I was solving the wrong problem.

The prediction model was never intended to perfectly simulate UAV dynamics.

Its only responsibility was generating meaningful residuals.

Once I understood that, the solution became much simpler.

A lightweight constant-velocity predictor is:

- deterministic,
- computationally inexpensive,
- easy to validate,
- and produces stable residuals suitable for continuous trust estimation.

Would a more sophisticated model improve prediction accuracy?

Probably.

Would it significantly improve localization integrity estimation?

Probably not.

Instead it would introduce additional parameters, additional tuning effort, more computational cost, and another source of debugging complexity.

That tradeoff wasn't worthwhile for the role this component actually plays inside the system.

Throughout TwinGuard I found myself making similar decisions repeatedly.

Whenever I had to choose between a simpler component with one clear responsibility and a more sophisticated component solving several problems simultaneously, I almost always chose the simpler architecture.

---

# Why I Separated Mission Planning from Safety

This decision appeared much later in development.

Initially it seemed perfectly reasonable for the Behavior Tree to publish PX4 commands directly.

After all, the planner already knew where the UAV should fly.

Adding another supervisory layer looked unnecessary.

The longer I worked on the project, the more uncomfortable I became with that architecture.

If the planner became responsible for both mission execution and safety enforcement, then every future planner modification could potentially bypass localization integrity constraints.

Mission logic and safety would become tightly coupled.

That violated one of the design principles I was gradually developing throughout the project.

Eventually I separated those responsibilities completely.

The Behavior Tree decides what the UAV should attempt to do.

The Offboard Supervisor decides whether those commands should actually reach PX4.

Every offboard command passes through the supervisor before publication.

Initially this felt like unnecessary complexity.

Today I think it is one of the cleanest architectural decisions in TwinGuard.

Mission planning remains focused entirely on behavior.

Safety enforcement remains centralized inside one location.

Whenever something unexpected happens during runtime, I always know exactly where authority decisions are being made.

That greatly simplified both debugging and future development.

---

# Why Everything Uses `trust_state`

As more packages were added, another problem slowly appeared.

Different components wanted different integrity information.

Some only needed the trust score.

Others only cared about authority scaling.

Some wanted residuals.

My first instinct was exposing multiple ROS topics.

The architecture quickly became fragmented.

Every new interface introduced another dependency between packages.

Every dependency made future changes slightly more difficult.

I eventually replaced those interfaces with one shared runtime contract:

```text
trust_state
```

Rather than representing a specific algorithm, the message represents the current localization confidence of the system.

Every major subsystem consumes exactly the same interface.

That includes:

- BehaviorTree mission supervision
- Offboard Supervisor
- Nav2 plugins
- Formation supervision
- Alternative integrity estimators

One lesson from this project has stayed with me ever since.

> **Stable interfaces are often more valuable than increasingly sophisticated algorithms.**

Once the `trust_state` interface stopped changing, the rest of the architecture became dramatically easier to extend.

I could improve the estimator without touching planners.

I could add Nav2 support without modifying supervision.

I could introduce dataset replay without changing integrity estimation.

The interface stayed constant while the implementation continued evolving.

Looking back, I think that was exactly the right tradeoff.

# Why I Designed the Kalman Pipeline as a Drop-In Replacement

Once the default integrity pipeline was working reliably, I started thinking about how additional sensing could improve localization confidence.

The obvious solution would have been replacing the original estimator entirely.

The more I thought about that approach, however, the less I liked it.

Replacing the estimator would force planners, supervisors, and navigation components to change simply because the implementation had changed. That completely contradicted the design philosophy I had gradually developed while building TwinGuard.

Instead, I treated the Kalman filter as another implementation of the same architectural interface.

The estimator fuses:

- PX4 vehicle odometry
- Sparse optical-flow visual odometry
- Depth-scaled motion estimates

From the perspective of the rest of the autonomy stack, nothing changes.

The planner still consumes `trust_state`.

The supervisor still consumes `trust_state`.

Nav2 still consumes `trust_state`.

Only the implementation producing that message changes.

This reinforced one of the biggest lessons I learned throughout the project.

Interfaces should remain stable even when implementations evolve.

That principle made later extensions dramatically easier than I originally expected.

---

# Why I Extended Nav2 Instead of Replacing It

When I began thinking about navigation, I never intended to build another navigation framework.

Nav2 already provides mature planners, controllers, and recovery behaviors.

Trying to replace that ecosystem would have required solving problems that had already been solved extremely well.

Instead I asked a much simpler question.

> **Can localization integrity become another input that Nav2 understands?**

That question naturally led to two lightweight integrations.

The first was a custom Behavior Tree condition allowing navigation logic to react to localization confidence.

The second was a localization-aware costmap layer.

Rather than representing physical obstacles, the layer represents uncertainty around the robot's own estimated position.

As localization confidence decreases, navigation becomes progressively more conservative without modifying Nav2's planners or controllers.

Looking back, extending mature software rather than replacing it was one of the better engineering decisions I made.

TwinGuard remains focused on localization integrity while Nav2 continues doing what it already does exceptionally well.

---

# Why I Built Dataset Replay

Validation became increasingly important as the project matured.

Injecting random Gaussian noise into localization was easy, but it never felt representative of how localization actually degrades.

I wanted experiments that were both realistic and repeatable.

Instead of replaying complete trajectories, I decided to replay localization degradation itself.

The replay node injects controlled perturbations into live PX4 odometry using measurements derived from real datasets.

From the perspective of the integrity pipeline, the vehicle behaves exactly like a live system experiencing degraded localization.

The remainder of the autonomy stack continues operating normally.

This turned out to be significantly more useful than synthetic disturbances because every experiment became deterministic.

Whenever I changed the estimator or supervisor, I could evaluate the exact same degradation profile and directly compare system behavior.

That made debugging considerably easier and gave me much greater confidence in the integrity pipeline.

---

# Why I Moved to a Microservice Deployment

Early versions of TwinGuard executed everything inside a single ROS 2 workspace.

That worked well while the project remained relatively small.

As additional components were introduced, the architecture became increasingly difficult to manage.

PX4, the autonomy runtime, Nav2, and experiment logging all had different responsibilities, yet restarting one component often meant restarting everything else.

Debugging also became more difficult because unrelated services shared the same execution environment.

Eventually I separated the project into independent runtime services connected through a Fast DDS Discovery Server.

Each service now performs one responsibility while communicating through ROS 2.

Besides making development significantly easier, this deployment model mirrors how larger robotics systems are commonly organized.

One lesson surprised me during this process.

Deployment architecture influences software architecture much more than I originally expected.

Once services became independent, their interfaces naturally became cleaner and responsibilities became much easier to define.

---

# Engineering Challenges

Most of the difficult problems I encountered while building TwinGuard were not algorithmic.

They were architectural.

Writing another estimator or another planner was relatively straightforward.

Deciding where responsibilities belonged was considerably harder.

One question repeatedly guided my design decisions.

> **Does this responsibility really belong inside this component?**

Whenever a node started accumulating multiple responsibilities, I usually found that introducing a shared interface produced a cleaner architecture.

That thought process eventually led to separating estimation, planning, supervision, and navigation into independent subsystems connected through `trust_state`.

Another challenge was resisting the temptation to allow components to depend directly on one another.

ROS 2 makes it very easy for nodes to exchange information.

That convenience can also encourage unnecessary coupling.

Throughout the project I deliberately tried to keep every package dependent only on stable runtime interfaces rather than implementation details.

Although this required more effort initially, it paid off every time I replaced or extended part of the system.

Balancing simplicity with capability became another recurring theme.

Many problems could have been addressed using increasingly sophisticated algorithms.

In most cases I deliberately chose simpler implementations with clearly defined responsibilities.

That decision made the architecture significantly easier to understand, validate, and extend.

---

# Lessons Learned

Looking back, I realized that the most valuable parts of TwinGuard are not individual algorithms.

They are the architectural principles that allowed those algorithms to evolve without requiring continuous redesign.

Several lessons became clear throughout development.

- Stable interfaces are often more valuable than increasingly sophisticated implementations.
- Separating responsibilities simplifies future extensions.
- Validation infrastructure is just as important as the autonomy algorithms themselves.
- Simplicity frequently produces more maintainable software than architectural complexity.
- Extending mature robotics software is often more valuable than replacing it.

These ideas continued supporting new functionality long after the original implementation was complete.

The Kalman estimator, Nav2 integration, dataset replay, and deployment architecture all benefited from decisions that were made much earlier in the project.

That reinforced my confidence that investing time in architecture early usually pays dividends later.

---

# Future Directions

Although the current architecture has remained remarkably stable, there are still several directions I would like to explore.

Future work includes:

- Multi-UAV trajectory conflict monitoring
- Hardware validation
- Additional localization sources
- More sophisticated integrity estimators
- Distributed multi-vehicle deployments

The encouraging part is that none of these extensions require significant architectural redesign.

Because every subsystem communicates through stable runtime interfaces, most future work can be added as new components rather than modifications to existing ones.

To me, that is probably the strongest evidence that the original architectural decisions were the right ones.

---

# Closing Thoughts

When I began building TwinGuard, I expected the algorithms to become the defining part of the project.

Instead, the architecture became the most valuable outcome.

The digital twin, Kalman filter, Behavior Tree, and dataset replay are all important pieces of the system, but none of them would have been as useful without an architecture that allowed them to work together while remaining loosely coupled.

If I were starting the project again today, I would certainly improve individual implementations and make different coding decisions based on what I have learned.

I would not fundamentally change the architecture.

Computing localization integrity once, exposing it through a shared runtime interface, separating mission planning from safety enforcement, and allowing every subsystem to evolve independently remain the engineering decisions I am most confident in.

Ultimately, TwinGuard taught me that resilient autonomy is not simply about detecting failures.

It is about designing systems that know how to respond as confidence changes.

More than any individual algorithm, that design philosophy is what I hope this project communicates.
