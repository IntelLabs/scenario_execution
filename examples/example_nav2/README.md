# Example Navigation

To run the Example Navigation 2 Scenario, first build the `tb4_sim_scenario` package:

```bash
colcon build --packages-up-to tb4_sim_scenario
```

Source the workspace:

```bash
source install/setup.bash
```

Now, run the following command to launch the scenario:

```bash
ros2 launch tb4_sim_scenario sim_nav_scenario_launch.py scenario:=examples/example_nav2/example_nav2.osc
```

A turtlebot is initialsed with nav2 which drives to a point and back.

For a more detailed understanding of the code structure and scenario implementation please refer to the [tutorial documentation](https://intellabs.github.io/scenario_execution/tutorials.html).
