# Example Moveit

To run the Example [moveit2](https://moveit.picknik.ai/main/index.html)  Scenario.

## 1. Update Submodules

```bash
git submodule update --init
```

## 2. Install Dependencies

```bash
rosdep install --from-paths . --ignore-src
```

## 3. Build Packages

```bash
colcon build --packages-up-to arm_sim_scenario
```

## 4. Source the Workspace

```bash
source install/setup.bash
```

## 5. Launch the Simulation

### a. Full Simulation

```bash
ros2 launch arm_sim_scenario sim_moveit_scenario_launch.py scenario:=examples/example_moveit/example_moveit.osc
```

### b.Visualization Only

```bash
ros2 launch arm_sim_scenario sim_moveit_scenario_launch.py scenario:=examples/example_moveit/example_moveit.osc ros2_control_hardware_type:=mock_components use_rviz:=true
```

For a more detailed understanding of the code structure and scenario implementation please refer to the [tutorial documentation](https://intellabs.github.io/scenario_execution/tutorials.html).
