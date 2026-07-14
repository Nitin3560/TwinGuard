# Quickstart

This guide walks through the minimum steps required to build and run TwinGuard on a clean Ubuntu system.

TwinGuard has been validated using:

- Ubuntu 24.04
- ROS 2 Jazzy
- PX4 SITL
- Gazebo Harmonic
- GitHub Actions continuous integration

---

# Prerequisites

Install the following software before building TwinGuard.

- ROS 2 Jazzy
- PX4 Autopilot
- Gazebo Harmonic
- Micro XRCE-DDS Agent
- `px4_msgs`
- `px4_ros_com`

Official PX4 documentation:

https://docs.px4.io/main/en/dev_setup/dev_env_linux_ubuntu.html

ROS 2 integration:

https://docs.px4.io/main/en/ros2/user_guide

---

# 1. Install PX4

Clone PX4 and install its development dependencies.

```bash
git clone https://github.com/PX4/PX4-Autopilot.git --recursive

cd PX4-Autopilot

bash ./Tools/setup/ubuntu.sh
```

Restart your terminal after installation completes.

---

# 2. Start PX4 SITL

Launch the default x500 multicopter in Gazebo Harmonic.

```bash
cd PX4-Autopilot

make px4_sitl gz_x500
```

Expected result:

```text
PX4 SITL starts successfully.

Gazebo launches with the x500 vehicle.

Vehicle telemetry becomes available through px4_msgs.
```

---

# 3. Configure the ROS 2 Bridge

Start the PX4 ROS 2 communication bridge.

Required components:

```text
px4_msgs
px4_ros_com
Micro XRCE-DDS Agent
```

Refer to the official PX4 ROS 2 documentation for bridge configuration.

---

# 4. Clone TwinGuard

Create a ROS 2 workspace.

```bash
mkdir -p ~/twinguard_ws/src

cd ~/twinguard_ws/src

git clone https://github.com/Nitin3560/TwinGuard.git
```

Import external repositories.

```bash
vcs import . < TwinGuard/dependencies.repos
```

---

# 5. Install Dependencies

```bash
cd ~/twinguard_ws

source /opt/ros/jazzy/setup.bash

rosdep install \
    --from-paths src \
    --ignore-src \
    -r \
    -y
```

---

# 6. Build TwinGuard

```bash
colcon build --symlink-install
```

When the build completes successfully,

source the workspace.

```bash
source install/setup.bash
```

---

# 7. Run the Test Suite

TwinGuard includes automated GoogleTest unit tests and ROS 2 integration tests.

```bash
colcon test

colcon test-result --verbose
```

A successful test run reports:

- TrustScorer tests
- Kalman Estimator tests
- A* Planner tests
- ROS 2 launch integration tests

The same workflow is executed automatically in GitHub Actions on every push.

---

# 8. Launch TwinGuard

Start the integrity pipeline.

```bash
ros2 launch twinguard_swarm_bringup \
    twinguard_integrity.launch.py
```

The launch starts:

- Integrity Node
- Formation Supervisor

---

# 9. Verify Runtime Topics

The integrity node subscribes to:

```text
/drone_0/fmu/out/vehicle_odometry
```

Expected outputs:

```text
/drone_0/twinguard/trust_state

/drone_0/twinguard/integrity_diagnostics

/drone_0/twinguard/supervisor_diagnostics
```

You can inspect the topics using:

```bash
ros2 topic list

ros2 topic echo /drone_0/twinguard/trust_state
```

---

# Expected Runtime

A successful launch should produce the following runtime sequence.

```text
PX4 SITL
      │
      ▼
VehicleOdometry
      │
      ▼
Integrity Node
      │
      ▼
Trust State
      │
      ▼
Formation Supervisor
      │
      ▼
Authority-scaled PX4 Commands
```

If PX4 telemetry is unavailable, the integrity node reports a waiting/stale diagnostic instead of publishing a healthy trust estimate.

---

# Next Steps

After verifying the integrity pipeline:

- Run dataset replay experiments
- Launch the Kalman integrity pipeline
- Enable Nav2 integration
- Record trust-aware PX4 SITL demonstrations

See the remaining documentation for:

- Architecture
- Engineering Design Decisions
- Topic Contract
- Deployment
- Dataset Replay
