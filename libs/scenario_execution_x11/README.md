# Scenario Execution Library for X11

The `scenario_execution_x11` package provides actions to interact with the X window system.

It provides the following scenario execution library:

- `x11.osc`: Actions specific to x11 window system


## Run Example

``` bash
colcon build
sudo apt install mesa-utils
ros2 run scenario_execution_ros scenario_execution_ros  libs/scenario_execution_x11/scenarios/example_capture.osc -t
```