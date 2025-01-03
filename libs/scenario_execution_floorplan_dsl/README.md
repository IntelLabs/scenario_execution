# Scenario Execution Library for Floorplan DSL

The `scenario_execution_floorplan_dsl` package provides actions to interact with [Floorplan DSL](https://github.com/secorolab/FloorPlan-DSL).

It provides the following scenario execution library:

- `floorplan_dsl.osc`: Floorplan DSL specific actions to create simulation worlds

## Build

``` bash
# build docker image of floorplan dsl
git clone https://github.com/secorolab/FloorPlan-DSL
cd FloorPlan-DSL
git checkout 34bd70bc89b285173226e27add6f4f5589de106a
docker build . --tag floorplan:latest

# build scenario execution and dependencies
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to scenario_execution_floorplan_dsl scenario_execution_ros scenario_execution_gazebo scenario_execution_nav2 tb4_sim_scenario
source install/setup.bash
```

## Test

``` bash
# run example scenario
ros2 run scenario_execution_ros scenario_execution_ros libs/scenario_execution_floorplan_dsl/example/example.osc -t
```