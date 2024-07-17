# Example Navigation

To run the example multi robot scenario, build the `tb4_sim_scenario` package:

```bash
colcon build --packages-up-to tb4_sim_scenario
```

Source the workspace:

```bash
source install/setup.bash
```
The Following example spawns 2 robots. The first robot is initialised with nav2 and moves in a straight line. Once it reaches a specific point a velocity command is sent to the second robot which will position itsel in front of the first one. The first robot then replans its path and moves around the second robot.

To actually run the scenario, run the following command.

```bash
ros2 launch tb4_sim_scenario sim_nav_scenario_launch.py scenario:=examples/example_multi_robot/example_multi_robot.osc
```

For a more detailed understanding of the code structure and scenario implementation please refer to the [tutorial documentation](https://intellabs.github.io/scenario_execution/tutorials.html).
