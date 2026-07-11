# Environment Setup

This document lists the software, tools, and dependencies required to build and run TwinGuard.

The project has primarily been developed and tested using Ubuntu, ROS 2 Jazzy, PX4 SITL, and Gazebo Harmonic.

---

# Operating System

The recommended development environment is:

- Ubuntu 24.04 LTS

Ubuntu 22.04 may also work with compatible ROS 2 and PX4 versions, but Ubuntu 24.04 is the primary target environment.

---

# Required Software

Install the following components before building the workspace.

| Software | Version |
|----------|---------|
| ROS 2 | Jazzy |
| PX4 Autopilot | Stable |
| Gazebo | Harmonic |
| Fast DDS | Compatible with ROS 2 Jazzy |
| Micro XRCE-DDS Agent | Latest |
| Docker | Latest |
| Docker Compose | Latest |
| CMake | 3.22+ |
| Python | 3.10+ |
| Git | Latest |

---

# ROS 2 Packages

TwinGuard depends on standard ROS 2 packages together with Nav2 and BehaviorTree.CPP.

Important packages include:

- Nav2
- BehaviorTree.CPP
- px4_msgs
- geometry_msgs
- nav_msgs
- sensor_msgs
- tf2
- Eigen3
- OpenCV

Any additional package dependencies are automatically resolved during the workspace build.

---

# Workspace

The repository follows a standard ROS 2 workspace structure.

```
ros2_ws/
    src/
```

All TwinGuard packages are located inside the `src` directory.

---

# Simulation Environment

TwinGuard currently targets:

- PX4 SITL
- Gazebo Harmonic
- ROS 2 Jazzy

The default simulation platform uses the PX4 x500 vehicle model.

---

# Optional Components

Some features require additional components.

### EKF Integrity Pipeline

Requires:

- RGB camera
- Depth camera
- OpenCV

---

### Nav2 Integration

Requires:

- Nav2
- TwinGuard Nav2 plugins

---

### Docker Deployment

Requires:

- Docker
- Docker Compose
- Fast DDS Discovery Server

---

# Verify Installation

Before building TwinGuard, verify the following:

- ROS 2 is installed and sourced.
- PX4 SITL launches successfully.
- Gazebo Harmonic is available.
- `px4_msgs` builds correctly.
- Docker is installed (optional for containerized deployment).

Once these components are available, continue with the Quick Start guide.
