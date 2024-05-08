# Use the ROS Humble desktop-full image as the base
FROM osrf/ros:humble-desktop-full


RUN apt-get update &&  \
    apt install -y python3-pip && \
    xargs -a deb_requirements.txt apt install -y --no-install-recommends && \
    rosdep update --rosdistro=humble && \
    rosdep install --rosdistro=humble --from-paths . --ignore-src -r -y && \
    pip3 install -r requirements.txt