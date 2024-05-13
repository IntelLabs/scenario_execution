# Use the ROS Humble desktop-full image as the base
FROM osrf/ros:humble-desktop-full

COPY . /scenario_execution/

RUN apt-get update &&  \
    apt-get install -y python3-pip && \
    apt-get install -y xvfb tk && \
    apt-get install -y libgl1 && \
    xargs -a /scenario_execution/deb_requirements.txt apt install -y --no-install-recommends && \
    rosdep update --rosdistro=humble && \
    rosdep install --rosdistro=humble --from-paths /scenario_execution/. --ignore-src -r -y && \
    pip3 install -r /scenario_execution/requirements.txt