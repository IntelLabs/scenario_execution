# Example Manipulation

To run the Example [moveit2](https://moveit.picknik.ai/main/index.html)  Scenario.

```bash
colcon build --packages-up-to arm_sim_scenario
```

Source the workspace:

```bash
source install/setup.bash
```

Now, run the following command to launch the scenario:

a. Full Simulation

```bash
ros2 launch arm_sim_scenario sim_moveit_scenario_launch.py scenario:=examples/example_moveit2/example_moveit2.osc
```

b.Visualization Only

```bash
ros2 launch arm_sim_scenario sim_moveit_scenario_launch.py scenario:=examples/example_moveit2/example_moveit2.osc ros2_control_hardware_type:=mock_components use_rviz:=true
```

The arm initially moves to a specified joint position. Next, the gripper opens. Once the gripper is open, the arm moves to the designated end-effector position. Finally, the gripper closes.

For a more detailed understanding of the code structure and scenario implementation please refer to the [tutorial documentation](https://intellabs.github.io/scenario_execution/tutorials.html).
