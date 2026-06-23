# PX4 image tags change across releases; verify the exact px4io tag before relying
# on this image in CI or hardware-facing validation.
FROM px4io/px4-dev-simulation-jazzy:latest

SHELL ["/bin/bash", "-c"]

WORKDIR /workspace
RUN git clone https://github.com/PX4/PX4-Autopilot.git --recursive

WORKDIR /workspace/PX4-Autopilot
RUN make px4_sitl_default

CMD ["make", "px4_sitl", "gz_x500_depth"]
