# Scenario Execution Library for Floorplan DSL

The `scenario_execution_floorplan_dsl` package provides actions to interact with [Floorplan DSL](https://github.com/secorolab/FloorPlan-DSL).

It provides the following scenario execution library:

- `floorplan_dsl.osc`: Floorplan DSL specific actions to create simulation worlds


## Test

```
colcon build -packages-up-to scenario_execution_floorplan_dsl scenario_execution_ros scenario_execution_gazebo tb4_sim_scenario
source /opt/ros/humble/setup.bash
```