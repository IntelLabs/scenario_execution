#!/bin/bash

echo "Starting DevCon from context: $PWD"

docker run -it --rm \
    --privileged \
    --net host \
    --ipc host \
    --pid host \
    --volume ${PWD}:/workspace \
    --volume /tmp/.X11-unix:/tmp/.X11-unix \
    --env DISPLAY=$DISPLAY \
    --env "ROS_DOMAIN_ID=$ROS_DOMAIN_ID" \
    --name scenario-execution \
    ger-registry-pre.caas.intel.com/amsrl/amsrl_humble_scenario_execution_devcontainertest:latest \
    $*
