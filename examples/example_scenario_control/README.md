# Example Navigation

To run the Example Scenario Control, first build the `example_scenario_control` Package:

```bash
colcon build --packages-up-to example_scenario_control
```

Source the workspace:

```bash
source install/setup.bash
```

Now, run the following command to launch the scenario:

```bash
ros2 launch example_scenario_control example_scenario_control_launch.py
```

For a more detailed understanding of the code structure and scenario implementation please refer to the [tutorial documentation](https://intellabs.github.io/scenario_execution/tutorials.html).

