ARG ROS_DISTRO=humble

FROM ros:${ROS_DISTRO}-ros-base
ENV TZ=Europe/Berlin
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

##############################################################################
##                                 Global Dependencies                       ##
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
    libgl1-mesa-glx \
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
# hadolint ignore=DL3008
RUN --mount=type=bind,target=/tmp_setup apt-get update \
    && xargs -a "/tmp_setup/.devcontainer/deb_requirements_${ROS_DISTRO}.txt" apt-get install -y --no-install-recommends "ros-${ROS_DISTRO}-rosbag2-storage-mcap" \
    && rm -rf /var/lib/apt/lists/*
    
# hadolint ignore=DL3013
RUN --mount=type=bind,target=/tmp_setup \
    python3 -m pip install --no-cache-dir --upgrade pip && \
    pip3 install --no-cache-dir -r /tmp_setup/requirements.txt

##############################################################################
##                           Create Non-root User                           ##
##############################################################################
ARG USER=ros
ARG PASSWORD=ros
ARG UID=1000
ARG GID=1000
ENV UID=${UID}
ENV GID=${GID}
ENV USER=${USER}
RUN groupadd -g "$GID" "$USER"  && \
    useradd -l -m -u "$UID" -g "$GID" --shell "$(which bash)" "$USER" -G sudo && \
    echo "$USER:$PASSWORD" | chpasswd && \
    echo "%sudo ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/sudogrp

RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /etc/bash.bashrc
USER $USER


##############################################################################
##                                 Build ROS and run                        ##
##############################################################################

WORKDIR /workspace
RUN . "/opt/ros/$ROS_DISTRO/setup.sh" \
    && colcon build 
    # --symlink-install

RUN printf "if [ -f /workspace/install/setup.bash ]; then \n\
              source /workspace/install/setup.bash \n\
          else \n\
              echo \"/workspace/install/setup.bash not found\" \n\
          fi" >> /home/$USER/.bashrc

# Switch to cyclonedds
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV IGN_FILE_PATH=/workspace/install/tb4_sim_scenario/share/
