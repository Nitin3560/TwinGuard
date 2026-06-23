# Quickstart

This quickstart describes the intended setup flow for a Linux robotics environment.

## 1. Install PX4 Development Dependencies

Follow the official PX4 setup guide for your Ubuntu version:

```text
https://docs.px4.io/main/en/dev_setup/dev_env_linux_ubuntu.html
```

Clone PX4:

```bash
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot
bash ./Tools/setup/ubuntu.sh
```

Restart the shell after dependency installation.

## 2. Run PX4 Gazebo SITL

```bash
cd PX4-Autopilot
make px4_sitl gz_x500
```

Expected result:

```text
PX4 SITL starts with an x500 multicopter in Gazebo.
```

## 3. Set Up ROS 2 / PX4 Bridge

Follow the official PX4 ROS 2 guides:

```text
https://docs.px4.io/main/en/ros2/user_guide
https://docs.px4.io/main/en/ros2/offboard_control
```

Core components:

```text
px4_msgs
px4_ros_com
Micro XRCE-DDS Agent
```

## 4. Build TwinGuard ROS 2 Workspace

```bash
cd TwinGuard-Swarm-Gazebo/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

For a reproducible container build path, see [deployment.md](deployment.md). The deployment Dockerfile consolidates the ROS, PX4 message, BehaviorTree.CPP, Nav2, Eigen, OpenCV, and Gazebo bridge dependencies used by the C++ packages.

## 5. Launch Initial TwinGuard Nodes

```bash
ros2 launch twinguard_swarm_bringup twinguard_integrity.launch.py
```

The launch starts the C++ integrity node and the initial formation supervisor. The C++ node expects PX4 odometry on:

```text
/drone_0/fmu/out/vehicle_odometry
```

Expected diagnostic outputs:

```text
/drone_0/twinguard/integrity_diagnostics
/drone_0/twinguard/trust_state
```

If PX4 odometry is not available, the integrity node reports a stale/waiting diagnostic instead of publishing a healthy trust score.
