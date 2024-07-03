# Use the ROS Humble desktop-full image as the base
FROM osrf/ros:humble-desktop-full

# Install system dependencies
# hadolint ignore=DL3008
RUN --mount=type=bind,source=.,target=/scenario_execution \
    apt-get update && \
    apt-get install -y --no-install-recommends \
        python3-pip \
        xvfb \
        tk \
        libgl1 && \
    xargs -a /scenario_execution/deb_requirements.txt apt-get install -y --no-install-recommends && \
    rosdep update --rosdistro=humble && \
    rosdep install --rosdistro=humble --from-paths /scenario_execution/ --ignore-src -r -y && \
    rm -rf /var/lib/apt/lists/*

# hadolint ignore=DL3042
RUN --mount=type=bind,source=.,target=/scenario_execution \
    pip3 install -r /scenario_execution/requirements.txt && \
    pip3 install -r /scenario_execution/libs/scenario_execution_floorplan_dsl/requirements.txt
