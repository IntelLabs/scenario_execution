# Example Navigation

To run the Example Navigation 2 Scenario, first build the `example_nav2` package:

```bash
colcon build --packages-up-to example_nav2
```

Source the workspace:

```bash
source install/setup.bash
```

Now, run the following command to launch the scenario:

```bash
ros2 launch example_nav2 example_nav2_launch.py
```

A turtlebot is initialsed with nav2 which drives to the scenario-specified goal.

For a more detailed understanding of the code structure and scenario implementation please refer to the [tutorial documentation](https://intellabs.github.io/scenario_execution/tutorials.html).
