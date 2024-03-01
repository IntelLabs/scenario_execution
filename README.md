# Scenario Execution

[![Super-Linter](https://github.com/IntelLabs/Scenario_Execution/actions/workflows/scan.yml/badge.svg)](https://github.com/marketplace/actions/super-linter)

## TL;DR

Scenario execution is a backend- and middleware-agnostic library written in Python based on the generic scenario description language [OpenSCENARIO 2](https://www.asam.net/static_downloads/public/asam-openscenario/2.0.0/welcome.html) and [pytrees](https://py-trees.readthedocs.io/en/devel/).
It reads a scenario definition from a file and then executes it, reusing available checks and behaviors. It is easily extendable through a library mechanism.
This separation of the scenario definition from implementation massively reduces the manual efforts of scenario creation.

To give an impression of the functionality of scenario execution, the following animation shows an example scenario with a turtlebot-like robot in simulation using Nav2 to navigate towards a specified navigation goal in a simulated warehouse environment.
Once the robot reaches a reference position and a box is spawned in front of the robot as an unmapped static obstacle that needs to be avoided.
Upon arrival of the goal position, the scenario is ends with success and the simulation gets cleaned up.

![scenario execution in action](docs/images/scenario.gif "in action")

## Setup

### Installation from source as ROS 2 workspace

Clone this repository, update its submodules by running:

```bash
git submodule update --init
```

install the necessary dependencies:

```bash
rosdep install  --from-paths . --ignore-src
pip3 install -r requirements.txt
```

and build it

```bash
colcon build
```

## How to run

First, build the packages:

```bash
colcon build
source install/setup.bash
```

To launch a scenario with ROS2:

```bash
ros2 launch scenario_execution scenario_launch.py scenario:=examples/example_scenario/scenarios/example_log.osc live_tree:=True
```
