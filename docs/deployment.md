# Deployment Realism

This document defines the deployment-facing path for TwinGuard. It adds reproducible container artifacts and a Jetson-oriented validation plan without claiming hardware validation before it has been run.

## Container Profiles

The existing `docker-compose.yaml` remains the simple development profile. It uses host networking and is still the fastest way to open a ROS 2 shell:

```bash
docker compose run --rm ros2-dev
```

The deployment profile is `docker-compose.microservices.yaml`. It uses a bridge network plus a Fast-DDS discovery server so ROS 2 services can discover one another without Docker host networking:

```bash
docker compose -f docker-compose.microservices.yaml up --build
```

The service boundaries follow coupling:

- `px4-sitl`: simulation boundary for PX4/Gazebo.
- `autonomy-core`: TwinGuard integrity, estimation, planning, and offboard supervision in one container to avoid adding DDS hops inside the safety-critical loop.
- `nav2-stack`: Nav2-facing integration as a distinct, swappable subsystem.
- `experiment-logger`: rosbag logging, intentionally independent from flight control.
- `discovery-server`: Fast-DDS rendezvous point for network-isolated ROS 2 containers.

This is a microservice deployment profile, not a claim that every ROS node should be isolated in its own container.

## DDS Discovery

ROS 2 DDS multicast discovery does not reliably cross Docker's default bridge network. The microservice compose file therefore uses Fast-DDS discovery-server mode:

```text
ROS_DISCOVERY_SERVER=discovery-server:11811
ROS_DOMAIN_ID=0
```

Before running TwinGuard itself, validate discovery with two trivial ROS 2 containers, one publishing `demo_nodes_cpp talker` and one running `demo_nodes_cpp listener`, both pointed at the same discovery server.

## Docker Images

`docker/ros2_ws/Dockerfile` consolidates the dependencies discovered across Phases 1-4:

- ROS 2 Jazzy desktop base image
- `px4_msgs`
- BehaviorTree.CPP v3
- Nav2 bringup and costmap plugin dependencies
- Eigen, OpenCV, `cv_bridge`
- `ros_gz_image` and `ros_gz_bridge`
- `colcon`

`docker/px4_sitl/Dockerfile` is intentionally marked as requiring PX4 image-tag verification before serious use. PX4 Docker image names and tags change over time, so treat that file as a deployment scaffold until the exact `px4io` tag is confirmed on Docker Hub.

## Gazebo Rendering

The multi-container profile is intended for headless validation. Gazebo GUI rendering inside Docker requires X11 or GPU passthrough and host permissions; keep visual demo recording on the simpler local workstation path unless the validation specifically needs a headed simulator in Docker.

## Jetson Orin Nano Plan

The deployment target for embedded realism is a single Jetson Orin Nano Super-class companion computer running TwinGuard ROS 2 containers while PX4 SITL and Gazebo can remain on a workstation.

Staged plan:

1. Flash JetPack 6.x with NVIDIA SDK Manager.
2. Install Docker on the Jetson; do not install ROS 2 natively unless a container path fails for a specific reason.
3. Run a trivial ROS 2 container smoke test on the Jetson.
4. Build `docker/ros2_ws/Dockerfile` on the Jetson to confirm the workspace builds on `aarch64`.
5. Connect the Jetson container to a workstation PX4/Gazebo SITL instance over the network.

The measurement that makes this a real engineering claim is latency, not just boot success. Record callback-to-publish latency for the integrity path and report p99 timing on Jetson versus workstation.

## Validation Order

Run validation in layers:

1. Native or single-container `colcon build` first.
2. Existing host-network dev compose regression.
3. Fast-DDS discovery smoke test across two trivial containers.
4. Full `docker-compose.microservices.yaml` topic-discovery check.
5. SITL integration.
6. Jetson smoke test and latency benchmark after the no-cost validation steps pass.
