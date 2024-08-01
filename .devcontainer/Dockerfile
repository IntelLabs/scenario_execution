ARG ROS_DISTRO=humble

FROM osrf/ros:${ROS_DISTRO}-desktop-full
ENV TZ=Europe/Berlin
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

##############################################################################
##                                 Global Dependencies                      ##
##############################################################################
# Install libraries for rendering
# hadolint ignore=DL3008
RUN DEBIAN_FRONTEND=noninteractive \
    apt-get update \
    && apt-get install -y -qq --no-install-recommends \
    libglvnd0 \
    libgl1 \
    libglx0 \
    libegl1 \
    libxext6 \
    libx11-6 \
    mesa-utils \
    libglx-mesa0 \
    libglu1-mesa-dev \
    freeglut3-dev \
    mesa-common-dev \
    "ros-${ROS_DISTRO}-rmw-cyclonedds-cpp" \
    "ros-${ROS_DISTRO}-ros-gz-interfaces" \
    python3-pip && \
    rm -rf /var/lib/apt/lists/*

COPY .devcontainer/ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh
##############################################################################
##                      Scenario Execution Dependencies                     ##
##############################################################################

RUN --mount=type=bind,target=/tmp_setup export DEBIAN_FRONTEND=noninteractive && \
    apt-get update && \
    xargs -a /tmp_setup/deb_requirements.txt apt-get install -y --no-install-recommends && \
    xargs -a /tmp_setup/libs/scenario_execution_kubernetes/deb_requirements.txt apt-get install -y --no-install-recommends && \
    rosdep update --rosdistro="${ROS_DISTRO}" && \
    for d in /tmp_setup/*; do \
        [[ ! -d  "$d" ]] && continue; \
        [[ "$(basename $d)" =~ ^(install|build|log)$ ]] && continue; \
        rosdep install --rosdistro="${ROS_DISTRO}" --from-paths "$d" --ignore-src -r -y; \
    done && \
    rm -rf /var/lib/apt/lists/*

##############################################################################
##                           Create Non-root User                           ##
##############################################################################
ARG USER=ros
ARG PASSWORD=ros
ARG UID=1234
ARG GID=1234
ENV UID=${UID}
ENV GID=${GID}
ENV USER=${USER}
RUN groupadd -g "$GID" "$USER"  && \
    useradd -l -m -u "$UID" -g "$GID" --shell "$(which bash)" "$USER" -G sudo && \
    echo "$USER:$PASSWORD" | chpasswd && \
    echo "%sudo ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/sudogrp

##############################################################################
##                           Setup ROS environment                          ##
##############################################################################

RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /etc/bash.bashrc
USER $USER
WORKDIR /workspace
RUN printf "if [ -f /workspace/install/setup.bash ]; then \n\
              source /workspace/install/setup.bash \n\
          else \n\
              echo \"/workspace/install/setup.bash not found\" \n\
          fi" >> /home/$USER/.bashrc

# Switch to cyclonedds
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
