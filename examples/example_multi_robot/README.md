# Example Navigation

To run the example multi robot scenario, first build the Package:

```bash
colcon build --packages-up-to example_multi_robot
```

Source the workspace:

```bash
source install/setup.bash
```
The Following example spawns 2 robots. The first robot is initialised with nav2 and moves in a straight line. Once it reaches a specific point a velocity command is sent to the second robot which will position itsel in front of the first one. The first robot then replans its path and moves around the second robot.

To actually run the scenario, run the following commands.
Note: due to an [issue](https://github.com/turtlebot/turtlebot4_simulator/issues/60) with the turtlebot simulation when running multiple robots, some launch commands need to run in separate terminals.
Launch the simulation and spawn the first robot by running the following command:

```bash
ros2 launch tb4_bringup sim_nav_scenario_launch.py scenario_execution:=False scenario:=foo yaw:=3.14
```

To spawn the second robot, run the following command in a new terminal:

```bash
ros2 launch tb4_bringup ignition_robot_launch.py namespace:=turtlebot2 x:=-3.0 y:=1.5 yaw:=-1.57
```

To run the actual scenario, run the following command in a third terminal:

```bash
ros2 launch scenario_execution scenario_launch.py scenario:=examples/example_multi_robot/scenarios/example_multi_robot.osc
```

For a more detailed understanding of the code structure and scenario implementation please refer to the [tutorial documentation](https://intellabs.github.io/scenario_execution/tutorials.html).
