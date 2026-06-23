FROM osrf/ros:jazzy-desktop

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
    git \
    python3-colcon-common-extensions \
    libeigen3-dev \
    libopencv-dev \
    ros-jazzy-ament-index-cpp \
    ros-jazzy-behaviortree-cpp-v3 \
    ros-jazzy-cv-bridge \
    ros-jazzy-nav2-bringup \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-ros-gz-image \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace/ros2_ws/src
RUN git clone https://github.com/PX4/px4_msgs.git -b release/1.15

COPY ros2_ws/src/ /workspace/ros2_ws/src/

WORKDIR /workspace/ros2_ws
RUN source /opt/ros/jazzy/setup.bash && colcon build --symlink-install

RUN printf '%s\n' \
    '#!/bin/bash' \
    'set -e' \
    'source /opt/ros/jazzy/setup.bash' \
    'source /workspace/ros2_ws/install/setup.bash' \
    'exec "$@"' \
    > /twinguard_entrypoint.sh && chmod +x /twinguard_entrypoint.sh

ENTRYPOINT ["/twinguard_entrypoint.sh"]
CMD ["bash"]
