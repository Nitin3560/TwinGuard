# Design Doc

This document explains the reasoning behind the architectural decisions I made while building TwinGuard. The architecture document describes how the system is organized, but that only tells half of the story. What I found much more interesting while working on this project was understanding *why* the architecture gradually evolved into its current form. Almost every major component changed at least once before settling into the design that exists today, and most of those changes happened because I encountered practical engineering problems rather than algorithmic ones.

When I started this project, I wasn't trying to build another PX4 simulation or another ROS 2 autonomy stack. My objective was to explore how localization integrity could become part of the decision-making process instead of remaining an isolated monitoring component. That single idea eventually influenced almost every architectural decision in the project. Looking back, I think the success of TwinGuard comes much more from the way the system is organized than from any individual algorithm inside it.

---

# Why I Made Integrity a Shared Signal

The first design decision I made was also the one that influenced the rest of the architecture the most. While looking at existing autonomy pipelines, I noticed that every subsystem simply assumed the localization estimate was correct. The planner trusted it, the controller trusted it, and the navigation stack trusted it as well. If localization quality degraded, every component either continued operating on unreliable information or reacted independently in different ways.

I wanted the entire autonomy stack to share one understanding of localization quality. Instead of asking whether the UAV had "failed," I wanted every subsystem to continuously know how trustworthy the current vehicle state was. That immediately ruled out a binary fault detector. A simple healthy-or-failed decision would force every downstream component to react in exactly the same way, which wasn't what I wanted. Some situations only require reducing vehicle authority, while others justify replanning or holding position. Those decisions become much easier when integrity is represented as a continuously varying quantity instead of a Boolean state.

That led me to designing a shared trust interface. Rather than allowing every package to estimate confidence independently, the integrity pipeline computes trust once and publishes it through a single topic. Planning, supervision, and navigation all consume exactly the same information. Besides making the architecture easier to understand, this decision dramatically reduced coupling between packages because integrity became a service that every component could use instead of something every component had to implement.

Looking back, this is probably the architectural decision I am happiest with because almost every extension I added later—the EKF pipeline, Nav2 integration, and dataset replay—benefited from keeping that interface stable.

---

# Why I Chose a Lightweight Digital Twin

One of the earliest questions I asked myself was how sophisticated the digital twin actually needed to be. It would have been easy to spend a large amount of time building a more detailed vehicle dynamics model, but the more I thought about the problem, the more I realized that perfect prediction was never the objective.

The digital twin exists for one purpose: producing meaningful residuals. As long as the prediction remains stable and deterministic, those residuals become a reliable indicator of localization quality. Increasing the complexity of the prediction model would certainly improve accuracy in some situations, but it would also introduce additional computational cost, more tuning parameters, and a system that would become harder to debug. None of those trade-offs significantly improved the integrity estimation itself.

I eventually settled on a lightweight constant-velocity predictor because it remained computationally inexpensive while generating consistent residuals in real time. The implementation became easier to understand, easier to validate, and easier to extend without sacrificing the actual purpose of the integrity pipeline. Throughout this project I found myself making similar decisions repeatedly—choosing simpler components with clearly defined responsibilities instead of pursuing maximum algorithmic complexity.

---

# Why I Separated Planning From Safety

This decision emerged naturally as I started implementing the Behavior Tree. At first, it seemed reasonable to let the planner publish commands directly to PX4. After all, the planner already knew where the UAV should fly, so allowing it to generate the final command looked like the simplest architecture.

The longer I worked on the system, the less comfortable I became with that approach. If the planner was responsible for both deciding the mission and enforcing safety, then future changes to planning logic could accidentally bypass integrity constraints. Safety would become tightly coupled to mission execution, making the architecture much harder to reason about.

Instead, I deliberately separated those responsibilities. The Behavior Tree decides what the UAV should attempt to do, whether that means continuing the nominal mission, performing a local reroute, or holding position. Those decisions never reach PX4 directly. Every candidate command must first pass through the Offboard Supervisor, which evaluates the current authority scale before publishing anything to the flight controller.

This extra layer initially felt unnecessary because it introduced another component into the runtime. After the project grew, however, it became one of the cleanest parts of the architecture. Mission planning remained focused entirely on behavior, while safety enforcement became centralized inside one location. That separation also made debugging significantly easier because I always knew where authority decisions were being made.

---

# Why Everything Uses `trust_state`

As more packages were added, I started noticing another problem. Different components wanted different pieces of integrity information. Some needed the trust score, others needed the residual, and others only cared about authority scaling. My first instinct was to expose multiple topics so each package could subscribe only to what it required.

That approach quickly became difficult to maintain. Every new feature introduced another interface, and every interface increased coupling between packages. Instead of simplifying the architecture, I was slowly making it more fragmented.

I decided to replace those interfaces with a single shared contract: `trust_state`. Rather than representing one specific algorithm, the message represents the outcome of the integrity pipeline. Every package consumes the same interface regardless of how the integrity estimate was generated. This turned out to be much more powerful than I originally expected because it allowed me to replace the default integrity node with the EKF implementation without changing the planner, the supervisor, or the Nav2 plugins.

This decision reinforced an important lesson for me. Stable interfaces are often more valuable than sophisticated algorithms. Once the interface became fixed, the rest of the architecture became significantly easier to evolve because improvements inside the integrity estimator no longer propagated throughout the entire project.

# Why I Designed the EKF as a Drop-In Replacement

After the default integrity pipeline was working, I started exploring whether additional sensing could improve localization confidence. The obvious solution would have been replacing the original integrity node entirely, but that would have forced every downstream component to change as well. The more I thought about it, the more I realized that would defeat the purpose of keeping the architecture modular.

Instead, I treated the EKF as another integrity estimator rather than a replacement for the architecture itself. The EKF combines PX4 position estimates with sparse optical-flow visual odometry and depth-scaled motion estimates to produce a more robust estimate of vehicle state. More importantly, it publishes exactly the same `trust_state` interface as the default integrity node. From the perspective of the planner, the supervisor, and Nav2, nothing changes. They simply consume a trust estimate without needing to know how that estimate was produced.

This ended up validating one of the original design goals of the project. By separating interfaces from implementations, I could improve the estimation pipeline without forcing changes throughout the rest of the autonomy stack.

---

# Why I Integrated Nav2 Instead of Replacing It

When I began thinking about navigation, I never wanted TwinGuard to become another navigation framework. Nav2 already provides mature planners, controllers, and recovery behaviors, and trying to replace that ecosystem would have added a huge amount of unnecessary work while solving a problem that had already been solved well.

Instead, I asked a much simpler question: *Can localization integrity become another input that Nav2 understands?*

That question led to two lightweight integrations. The first is a custom Behavior Tree condition that allows navigation logic to react to localization confidence. The second is a localization-aware costmap layer that represents uncertainty around the robot's own estimated position instead of external obstacles. As localization confidence decreases, the navigation stack naturally becomes more conservative without requiring any modifications to Nav2's existing planners or controllers.

Looking back, I think extending mature software instead of replacing it was one of the better engineering decisions I made. It allowed TwinGuard to remain focused on integrity while letting Nav2 continue doing what it already does extremely well.

---

# Why I Built the Dataset Replay Pipeline

Once the integrity pipeline was functioning correctly in simulation, I started thinking about validation. Injecting random Gaussian noise into localization was easy, but it didn't represent how localization actually degrades in the real world. I wanted experiments that were repeatable while still reflecting realistic sensing conditions.

Instead of replaying complete trajectories, I decided to replay localization degradation itself. The replay node injects controlled perturbations into live PX4 odometry using measurements derived from real datasets, allowing the rest of the autonomy stack to continue operating normally. From the perspective of the integrity pipeline, the vehicle behaves exactly like a live system experiencing degraded localization, while every experiment remains deterministic and repeatable.

This turned out to be much more valuable than simply adding random disturbances because I could evaluate exactly how trust, authority scaling, and mission supervision behaved under known degradation profiles.

---

# Why I Moved to a Microservice Deployment

When TwinGuard was still small, running everything inside a single ROS 2 workspace worked well enough. As the project grew, that approach became increasingly difficult to manage. PX4, the autonomy runtime, Nav2, and experiment logging all had different responsibilities, yet restarting one component often meant restarting unrelated parts of the system. Debugging also became more complicated because every service shared the same execution environment.

I eventually separated the project into independent runtime services connected through a Fast DDS Discovery Server. PX4, the TwinGuard runtime, Nav2, and experiment logging each run independently while communicating through ROS 2. Besides making development significantly easier, this architecture mirrors how larger robotics systems are commonly deployed and makes future distributed deployments much more practical.

The biggest lesson from this change was that deployment architecture influences software architecture much more than I originally expected. Once components became independent services, their interfaces naturally became cleaner and much easier to maintain.

---

# Engineering Challenges

Most of the difficult problems I encountered while building TwinGuard were not algorithmic. They were architectural. Writing another planner or another estimator was relatively straightforward compared to deciding how those components should communicate without becoming tightly coupled.

One challenge I repeatedly encountered was deciding where responsibilities should live. Every time a component started accumulating too many responsibilities, I asked myself whether that logic really belonged there or whether it should be moved behind a shared interface. That thought process is ultimately what led to separating estimation, planning, supervision, and navigation into independent subsystems connected through `trust_state`.

Another challenge was resisting the temptation to make components aware of each other's implementation details. It is very easy in ROS 2 to let one node become dependent on another because the information is readily available. I intentionally tried to avoid that wherever possible. Every package communicates through ROS topics or clearly defined interfaces, allowing individual components to evolve independently. This approach required more design effort initially, but it paid off later whenever I replaced or extended parts of the system without affecting the rest of the architecture.

Balancing simplicity with capability was another recurring decision throughout the project. Many problems could have been solved by introducing more sophisticated algorithms, but that often came at the cost of making the system harder to understand and maintain. Whenever possible, I preferred simpler solutions with clearly defined responsibilities over more complex implementations that solved several problems at once. I wanted someone reading the repository to understand why every major component existed before worrying about the details of how it worked.

# Looking Back

When I started TwinGuard, I thought the most difficult part of the project would be implementing the algorithms. I expected the digital twin, the EKF, and the planning logic to consume most of my time. While those components certainly required effort, I eventually realized that the real challenge was designing an architecture where those algorithms could work together without becoming tightly coupled.

The project taught me that good interfaces are often more valuable than sophisticated implementations. A better digital twin or a more advanced planner can always be added later, but if the architecture depends on every component knowing how every other component works, the project quickly becomes difficult to extend. Once I committed to computing trust once and exposing it through a single interface, many of the later design decisions became much easier. The EKF could replace the default estimator, Nav2 could consume localization confidence, and new components could be introduced without rewriting the rest of the autonomy stack.

Another lesson I learned was the importance of separating responsibilities. It was tempting in several places to let one component solve multiple problems simply because it already had access to the required information. The Behavior Tree could have published PX4 commands directly, the planner could have estimated integrity, or the integrity node could have contained supervision logic. Each of those approaches would probably have reduced the amount of code in the short term, but they would also have made the architecture much harder to understand and maintain. Keeping each subsystem focused on one responsibility required more discipline, but I think it produced a cleaner design in the long run.

I also gained a much greater appreciation for validation. Building an algorithm is only one part of an autonomy project. Understanding how it behaves under controlled, repeatable conditions is equally important. That realization led me to build the dataset replay pipeline instead of relying only on synthetic disturbances. Being able to reproduce the same degradation profile repeatedly made debugging easier and gave me much more confidence in the behavior of the trust pipeline.

Looking back at the repository today, there are still several things I would like to improve. Multi-agent trajectory conflict monitoring is still incomplete, additional validation on physical hardware would strengthen the project, and there is plenty of room to explore more advanced estimation techniques. Those improvements, however, feel like extensions of the architecture rather than changes to it. The core design has remained stable because most of the important architectural decisions were made early and have continued to support new functionality without requiring major redesign.

If I were starting TwinGuard again today, I would certainly write cleaner code in several places and make different implementation choices based on what I have learned. I would not change the overall architecture. Computing localization integrity once, treating trust as a continuous quantity instead of a binary decision, and keeping estimation, planning, supervision, and navigation independent while connecting them through a shared interface are still the decisions I am most confident in.

Ultimately, I think the biggest lesson from this project is that resilient autonomy is not just about detecting failures. It is about designing systems that know how to respond when confidence begins to change. The algorithms inside TwinGuard are important, but I believe the architecture is what makes those algorithms practical, maintainable, and extensible. More than anything else, that is the idea I hope this project communicates.
