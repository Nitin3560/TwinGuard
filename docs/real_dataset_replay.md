# Real Dataset Replay

TwinGuard-Swarm-Gazebo uses PX4/Gazebo for live 3D simulation and replays real dataset degradation profiles into the ROS 2 odometry stream. This keeps the visual scene simulator-driven while testing the integrity layer against real time-series behavior.

## Runtime Flow

```text
PX4 gz_x500 + Gazebo
        |
        v
/fmu/out/vehicle_odometry
        |
        v
twinguard_dataset_replay
        |
        v
/twinguard/replay/vehicle_odometry
        |
        v
twinguard_swarm_integrity_cpp
        |
        +--> /twinguard/integrity_diagnostics
        +--> /twinguard/trust_state
```

## Dataset Columns

The replay node accepts CSV files with any of these degradation columns:

```text
measurement_error_m
est_err_m
dt_err_m
sensor_residual_m
innovation_norm_m
nis
```

Optional quality/anomaly columns:

```text
quality
true_quality
is_outlier
ids_anomaly_flag
```

The node converts the dataset degradation profile into an odometry perturbation, then the C++ integrity node computes residual, trust, anomaly label, and authority scale.

## Launch

With PX4 SITL and Micro XRCE-DDS Agent already running:

```bash
source /opt/ros/humble/setup.bash
source ~/px4_ros2_ws/install/setup.bash

ros2 launch twinguard_swarm_bringup twinguard_dataset_replay_integrity.launch.py \
  dataset_csv:=/absolute/path/to/real_dataset.csv \
  error_scale:=0.1 \
  max_offset_m:=3.0
```

Watch diagnostics:

```bash
ros2 topic echo /twinguard/dataset_replay_diagnostics
ros2 topic echo /twinguard/integrity_diagnostics
ros2 topic echo /twinguard/trust_state
```

## Recording

Run Gazebo/PX4 with the GUI enabled when recording the 3D view:

```bash
cd ~/PX4-Autopilot
make px4_sitl gz_x500
```

Use the ROS 2 terminals to show the live dataset replay and TwinGuard diagnostics while recording the Gazebo 3D scene.
