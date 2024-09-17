# Example Moveit

To run the Example Moveit 2 Scenario, first build the `arm_sim_scenario` package:

Update submodule

```bash
git submodule update --init
```

Install dependencies

```bash
rosdep install  --from-paths simulation/gazebo/arm_sim_scenario --ignore-src
```

build packages

```bash
colcon build --packages-up-to arm_sim_scenario
```

Source the workspace:

```bash
source install/setup.bash
```

Now, run the following command to launch the scenario:

```bash
ros2 launch arm_sim_scenario sim_moveit_scenario_launch.py scenario:=examples/example_moveit/example_moveit.osc
```


For a more detailed understanding of the code structure and scenario implementation please refer to the [tutorial documentation](https://intellabs.github.io/scenario_execution/tutorials.html).
