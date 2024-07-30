ARG ROS_DISTRO=humble

FROM osrf/ros:${ROS_DISTRO}-desktop-full
SHELL ["/bin/bash", "-xo", "pipefail", "-c"]

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
    rosdep update --rosdistro=${ROS_DISTRO} && \
    rosdep install --rosdistro=${ROS_DISTRO} --from-paths /scenario_execution/ --ignore-src -r -y && \
    rm -rf /var/lib/apt/lists/*

RUN --mount=type=bind,source=.,target=/scenario_execution \
    pip3 install  --no-cache-dir -r /scenario_execution/libs/scenario_execution_pybullet/requirements.txt

# install rosbag2_to_video used within github actions
WORKDIR /rosbag2_to_video/src
RUN git clone https://github.com/ivanpauno/rosbag2_to_video.git
WORKDIR /rosbag2_to_video/src/rosbag2_to_video
RUN git checkout 6bb3ada33aa2cb5c2ebd506f8c7ac22c39ec20d9
WORKDIR /rosbag2_to_video
RUN apt-get update && \
    rosdep install --from-paths src --ignore-src -y && \
    rm -rf "/var/lib/apt/lists/*" && \
    . "/opt/ros/${ROS_DISTRO}/setup.bash" && \
    colcon build

WORKDIR /scenario_execution
