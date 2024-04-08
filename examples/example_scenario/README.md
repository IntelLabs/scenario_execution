# Example Scenario

To run the Example Scenario, first build the `scenario_execution` package:

```bash
colcon build --packages-up-to scenario_execution
```

Source the workspace:

```bash
source install/setup.bash
```

Now, run the following command to launch the scenario:

```bash
ros2 run scenario_execution scenario_execution examples/example_scenario/hello_world.osc
```

For a more detailed understanding of the code structure and scenario implementation please refer to the [tutorial documentation](https://intellabs.github.io/scenario_execution/tutorials.html).
