# Use the ROS Humble desktop-full image as the base
FROM osrf/ros:humble-desktop-full

# Copy the repository into the container
COPY . /scenario_execution/

# Install system dependencies
# hadolint ignore=DL3008
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        python3-pip \
        xvfb \
        tk \
        libgl1 && \
    xargs -a /scenario_execution/deb_requirements.txt apt-get install -y --no-install-recommends && \
    rosdep update --rosdistro=humble && \
    rosdep install --rosdistro=humble --from-paths /scenario_execution/ --ignore-src -r -y && \
    # hadolint ignore=DL3042
    pip3 install -r /scenario_execution/requirements.txt && \
    rm -rf /var/lib/apt/lists/* && \
    rm -rf /scenario_execution/