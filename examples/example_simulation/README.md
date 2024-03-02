# Example Simulation Navigation

To run the Example Simulation Navigation with scenario, first build the Example Simulation Navigation Package:

```bash
colcon build --packages-up-to example_simulation
```

Source the workspace:

```bash
source install/setup.bash
```

Now, run the following command to launch the scenario:

```bash
ros2 launch tb4_sim_scenario sim_nav_scenario_launch.py scenario:=examples/example_simulation/scenarios/example_simulation.osc
```

A turtlebot is initialsed with nav2 which drives to a point and back. During the ride an object is spawned in front of the turtlebot which will then drive around the object.

For a more detailed understanding of the code structure and scenario implementation please refer to the [tutorial documentation](https://intellabs.github.io/scenario_execution/tutorials.html).
