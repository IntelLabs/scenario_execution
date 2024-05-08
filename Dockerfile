# Use the ROS Humble desktop-full image as the base
FROM osrf/ros:humble-desktop-full

COPY ../scenario_execution /scenario_execution/

RUN apt-get update &&  \
    apt install -y python3-pip && \
    ls  && \
    ls /scenario_execution && \
    xargs -a /scenario_execution/deb_requirements.txt apt install -y --no-install-recommends && \
    rosdep update --rosdistro=humble && \
    rosdep install --rosdistro=humble --from-paths . --ignore-src -r -y && \
    pip3 install -r /scenario_execution/requirements.txt