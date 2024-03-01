# Example Library

To run the Example Library Scenario, first build the Example Scenario Package:

```bash
colcon build --packages-up-to example_library
```

Source the workspace:

```bash
source install/setup.bash
```

Now, run the following command to launch the scenario:

```bash
ros2 launch scenario_execution scenario_launch.py scenario:=examples/example_library/scenarios/example_library.osc
```

For a more detailed understanding of the code structure and scenario implementation please refer to the [tutorial documentation](https://intellabs.github.io/scenario_execution/tutorials.html).

