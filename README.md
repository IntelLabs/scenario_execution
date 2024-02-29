# Scenario Execution

## TL;DR

Scenario execution is a backend- and middleware-agnostic library written in Python based on the generic scenario description language [OpenScenario2.0](https://www.asam.net/index.php?eID=dumpFile&t=f&f=3460&token=14e7c7fab9c9b75118bb4939c725738fa0521fe9) and [pytrees](https://py-trees.readthedocs.io/en/devel/).
It reads a scenario definition from a file and then executes it, reusing available checks and behaviors.
This separation of the scenario definition from implementation massively reduces the manual efforts of scenario creation.

To give an impression of the functionality of scenario execution, the following animation shows an example scenario with a turtlebot-like robot in simulation using Nav2 to navigate towards a specified navigation goal in a simulated warehouse environment.
Once the robot reaches a reference position and a box is spawned in front of the robot as an unmapped static obstacle that needs to be avoided.
Upon arrival of the goal position, the scenario is ends with success and the simulation gets cleaned up.

![scenario execution in action](Docs/scenario.gif "in action")

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

### With devcontainer

#### Preparations

If not already installed, install the docker engine on your system according to the [installation instructions](https://docs.docker.com/engine/install/) or, if you need GPU support, follow the [nvidia installation instructions](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html).
The recommended nvidia driver package is `nvidia-driver-525`.

Make sure you follow the [post installation steps](https://docs.docker.com/engine/install/linux-postinstall/).

Post-installation, you need to configure the proxy behaviour of the docker daemon:

```bash
sudo mkdir -p /etc/systemd/system/docker.service.d
sudo echo -e "[Service]\nEnvironment="HTTP_PROXY=http://proxy-chain.intel.com:911"\nEnvironment="HTTPS_PROXY=http://proxy-chain.intel.com:912"\nEnvironment="NO_PROXY=*.intel.com,127.0.0.0/8,localhost,127.0.0.1"" > /etc/systemd/system/docker.service.d/http-proxy.conf
sudo systemctl -q daemon-reload
sudo systemctl -q restart docker
```

To make sure, that the docker daemon is properly set up, run

```bash
docker run hello-world
```

Install additional packages to build amsrl containers (buildkit).

```bash
sudo apt install docker-buildx-plugin
```

Follow these [instructions](https://intel.sharepoint.com/sites/caascustomercommunity/SitePages/CaaS%20-%20Containers%20as%20a%20Service/News1915835.aspx#) for using the Intel image registry.

Particularly, install the Intel internal CAs following these [instructions](https://github.intel.com/CaaS/public/blob/master/ca_install.md).
Now, you should be ready to login to be able to pull docker images.

```bash
docker login ger-registry-pre.caas.intel.com
```

In case you want to use the devcontainer with [Visual Studio Code](https://code.visualstudio.com/), you also need to install docker-compose via

```bash
sudo apt update
sudo apt install docker-compose
```

#### Run devcontainer from terminal

Prior to the actual start of the container, run

```bash
xhost + local:ros
```

To start the devcontainer without GPU-support, run the following command *from the root directory of this repository*

```bash
bash containers/run_cpu.sh
```

To start the devcontainer with GPU-support, run the following command *from the root directory of this repository*

```bash
bash containers/run_gpu.sh
```

Inside the devcontainer, you can now safely run and test your development, e.g., run

```bash
ros2 run scenario_execution scenario_execution scenario_execution_base/scenarios/demo_wait_and_log.osc -t
```

For a more sophisticated example using a simulated [Turtlebot4](https://turtlebot.github.io/turtlebot4-user-manual/) and [Nav2](https://navigation.ros.org/), run

```bash
ros2 launch tb4_bringup sim_nav_scenario_launch.py scenario:=examples/example_nav2/scenarios/example_nav2.osc
```

and you should something like this

![turtlebot4 nav2 scenario](Docs/tb4_scenario.gif "turtlebot4 nav2 scenario")

In case you need an additional terminal inside your running devcontainer, run

```bash
bash containers/exec.sh
```

When you are done with testing, please run

```bash
xhost - local:ros
```

#### Run devcontainer from Visual Studio Code

Make sure you have installed the necessary VS Code extensions, namely the [docker extension](https://code.visualstudio.com/docs/containers/overview) as well as the [Dev Container](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers) extension.
Open the root folder of this repository in Visual Studio Code and click the blue item in the lower left corner

![vscode1](./Docs/graphs/vscode1.png "vscode1")

Afterwards, select "Reopen in Container " in the Selection Window inside VS Code

![vscode2](./Docs/graphs/vscode2.png "vscode2")

Now VS Code should open your current working directory inside the devcontainer.
If you now open a terminal inside VS Code, you can run and test your development safely inside the container, e.g., run

```bash
ros2 run scenario_execution scenario_execution scenario_execution_base/scenarios/demo_wait_and_log.osc -t
```

For a more sophisticated example using a simulated [Turtlebot4](https://turtlebot.github.io/turtlebot4-user-manual/) and [Nav2](https://navigation.ros.org/), run

```bash
ros2 launch scenario_execution_tutorials turtlebot4_simulation_nav2_to_pose_tutorial_launch.py headless:=False
```

and you should something like this

![turtlebot4 nav2 scenario](Docs/tb4_scenario.gif "turtlebot4 nav2 scenario")

Once you are done, you can cancel the remote connection, by again clicking on the blue item in the lower left corner and select "Close Remote Connection"

![vscode3](./Docs/graphs/vscode3.png "vscode3")

### With local installation

First, build the packages:

```bash
colcon build --packages-up-to scenario_execution_gazebo
source install/setup.bash
```

To launch an osc file with ROS2, use the default launch file:

```bash
ros2 launch scenario_execution scenario_launch.py scenario:=$(PATH_TO_SCENARIO_FILE) debug:=True log-level:=debug
```

To run an osc file with ROS2:

```bash
ros2 run scenario_execution scenario_execution $(PATH_TO_SCENARIO_FILE)
```

Use `-t` flag to see the printed tree and use `-d` flag to see debug information of py_trees and parser:

```bash
ros2 run scenario_execution scenario_execution $(PATH_TO_SCENARIO_FILE) -t -d
```

## Tutorials

Examples and tutorials on how to, for instance, create your own scenario files or how to implement custom plugins can be found in [scenario_execution_tutorials](scenario_execution_tutorials/README.md).

## Architecture overview

![scenario_execution_structure.png](Docs/graphs/scenario_execution_structure.png)

The scenario execution contains several sub-packages, namely

- [scenario_execution_base](#scenario-execution-base-package)
- [scenario_execution](#scenario-execution-package)
- [scenario_execution_gazebo](#scenario-execution-gazebo-package)
- [scenario_execution_control](#scenario-execution-control-package)
- [scenario_execution_interfaces](#scenario-execution-interfaces-package)
- [scenario_execution_rviz](#scenario-execution-rviz-package)
- [scenario_execution_kubernetes](#scenario-execution-kubernetes-package)

The architecture aims at modularity with each package implementing a specific functionality.

### Scenario Execution Base Package

The "scenario_execution_base" package is the base package for scenario execution. It provides functionalities like parsing and base classes. For more documentation of this package, please refer to [scenario_execution_base package README](scenario_execution_base/README.md).

### Scenario Execution Package

The "scenario_execution" package uses ROS2 as middleware and contains ROS2-specific modules.
For more documentation of this package, please refer to [scenario_execution package README](scenario_execution/README.md).

### Scenario Execution Gazebo Package

The "scenario_execution_gazebo" package is an extension of the "scenario_execution" package with Gazebo/AMR dependencies.
For more documentation of this package, please refer to [scenario_execution_gazebo package README](scenario_execution_gazebo/README.md).

### Scenario Execution Control Package

The "scenario_execution_control" package provides code to control scenarios (in ROS2) from another application such as Rviz.
For more documentation of this package, please refer to [scenario_execution_control package README](scenario_execution_control/README.md).

### Scenario Execution Interfaces Package

The "scenario_execution_interfaces" package provides ROS2 [interfaces](https://docs.ros.org/en/rolling/Concepts/Basic/About-Interfaces.html),  more specifically, messages and services, which are used to interface ROS2 with the [scenario execution control package](../scenario_execution_control/README.md)
For more documentation of this package, please refer to [scenario_execution_interfaces package README](scenario_execution_interfaces/README.md).

### Scenario Execution Rviz Package

The "scenario execution rviz" package contains code for several [rviz](https://github.com/ros2/rviz) plugins for visualizing and controlling scenarios when working with [ROS 2](https://docs.ros.org/en/rolling/index.html).
For more documentation of this package, please refer to [scenario_execution_rviz package README](scenario_execution_rviz/README.md).

### Scenario Execution Kubernetes Package

The "scenario_execution_kubernetes" package contains custom conditions and actions when running scenarios inside a [Kubernetes](https://kubernetes.io/) cluster.
For more documentation of this package, please refer to [scenario_execution_kubernetes package README](scenario_execution_kubernetes/README.md).


## Testing

To run only specific tests:

```bash
#using py-test
colcon build --packages-up-to scenario_execution && reset && pytest-3 -s scenario_execution/test/<TEST>.py 

#manual run
colcon build --packages-up-to scenario_execution && reset && ros2 launch scenario_execution scenario_launch.py scenario:=<...> debug:=True
```

## Contribute

- [Create additional Actions](Docs/create_action.md)

Before pushing your code, please ensure that the code formatting is correct by running:

```bash
make
```

In case of errors you can run the autoformatter by executing:

```bash
make format
```
