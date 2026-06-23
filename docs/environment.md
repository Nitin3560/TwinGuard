# Development Environment

TwinGuard-Swarm-Gazebo is designed for a Linux robotics development environment with ROS 2, Gazebo, and PX4 SITL.

## Recommended Platform

```text
Ubuntu 22.04 or Ubuntu 24.04
ROS 2 Humble or Jazzy
Gazebo / Gazebo Harmonic-compatible PX4 tooling
PX4-Autopilot
```

This environment provides the most direct path for PX4 SITL, Gazebo rendering, ROS 2 middleware, and DDS-based PX4 message exchange.

## Containerized Development

A containerized ROS 2 development environment is provided for package builds and headless checks:

```bash
docker compose run --rm ros2-dev
```

Inside the container:

```bash
source /opt/ros/jazzy/setup.bash
cd ros2_ws
colcon build --symlink-install
```

Containerized development is useful for validating ROS 2 package structure, C++ nodes, and Python experiment tooling. Full Gazebo rendering and PX4 SITL workflows are best validated in a native Ubuntu desktop or workstation environment.

## Deployment Containers

The deployment-oriented profile is documented in [deployment.md](deployment.md). It adds:

```text
docker/ros2_ws/Dockerfile
docker/px4_sitl/Dockerfile
docker-compose.microservices.yaml
```

Unlike the simple development compose file, the microservice profile uses a Docker bridge network and a Fast-DDS discovery server. This preserves container network isolation while avoiding the common ROS 2 failure mode where DDS multicast discovery does not cross Docker's default bridge network.

Use the deployment profile after the native or single-container build path has passed:

```bash
docker compose -f docker-compose.microservices.yaml up --build
```

## Deployment Profiles

### Local Ubuntu Workstation

Best for full simulator development, Gazebo visualization, ROS 2 tooling, and video capture.

### Remote Ubuntu Workstation

Recommended when a local Linux workstation is unavailable. Use SSH for development and remote desktop or streaming for Gazebo visualization.

### Cloud Ubuntu Instance

Useful for short validation runs, package builds, and headless PX4 SITL experiments. GPU-backed instances are only required for heavier rendering or future Isaac Sim workflows.
