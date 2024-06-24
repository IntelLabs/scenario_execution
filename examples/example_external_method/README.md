# Example External Method

To run the Example External Method Scenario, first build the package:

```bash
colcon build --packages-up-to example_external_method
```

Source the workspace:

```bash
source install/setup.bash
```

Now, run the following command to launch the scenario:

```bash
ros2 run scenario_execution scenario_execution examples/example_external_method/scenarios/example_external_method.osc
```

For a more detailed understanding of the code structure and scenario implementation please refer to the [tutorial documentation](https://intellabs.github.io/scenario_execution/tutorials.html).

