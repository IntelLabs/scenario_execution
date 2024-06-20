ARG ROS_DISTRO

FROM osrf/ros:${ROS_DISTRO}-desktop-full
SHELL ["/bin/bash", "-xo", "pipefail", "-c"]

# hadolint ignore=DL3008
RUN --mount=type=bind,source=.,target=/scenario_execution \
    apt-get update && \
    apt-get install -y --no-install-recommends \
        python3-pip \
        xvfb \
        tk \
        libgl1 && \
    xargs -a /scenario_execution/deb_requirements_${ROS_DISTRO}.txt apt-get install -y --no-install-recommends && \
    rosdep update --rosdistro=${ROS_DISTRO} && \
    rosdep install --rosdistro=${ROS_DISTRO} --from-paths /scenario_execution/ --ignore-src -r -y

# hadolint ignore=DL3042
RUN --mount=type=bind,source=.,target=/scenario_execution \
    if [[ "$ROS_DISTRO" == "humble" ]] ; then \
        pip3 install -r /scenario_execution/requirements.txt && \
        rm -rf /var/lib/apt/lists/* \
    ;fi

RUN --mount=type=bind,source=.,target=/scenario_execution \
    if [[ "$ROS_DISTRO" == "jazzy" ]] ; then \
        pip3 install --break-system-packages -r /scenario_execution/requirements.txt && \
        rm -rf /var/lib/apt/lists/* \
    ;fi
