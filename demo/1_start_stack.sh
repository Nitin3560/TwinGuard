#!/bin/bash
gnome-terminal --title="Gazebo" -- bash -c "cd ~/PX4-Autopilot && make px4_sitl gz_x500; exec bash"
sleep 8
gnome-terminal --title="Agent" -- bash -c "MicroXRCEAgent udp4 -p 8888; exec bash"
sleep 2
gnome-terminal --title="TwinGuard" -- bash -c "source ~/twinguard_ws/install/setup.bash && ros2 launch ~/twinguard_ws/src/TwinGuard/demo/demo_launch.py; exec bash"
sleep 2
gnome-terminal --title="RViz" -- bash -c "source /opt/ros/humble/setup.bash && rviz2; exec bash"
sleep 2
gnome-terminal --title="Plot" -- bash -c "source /opt/ros/humble/setup.bash && ros2 run rqt_plot rqt_plot /twinguard/trust_state/point/x /twinguard/trust_state/point/y /twinguard/trust_state/point/z; exec bash"
sleep 2
gnome-terminal --title="Readout" -- bash -c "source /opt/ros/humble/setup.bash && watch -n 0.2 'ros2 topic echo /twinguard/trust_state -n 1'; exec bash"
sleep 2
gnome-terminal --title="Recorder" -- bash -c "source /opt/ros/humble/setup.bash && python3 ~/twinguard_ws/src/TwinGuard/demo/record_metrics.py; exec bash"
