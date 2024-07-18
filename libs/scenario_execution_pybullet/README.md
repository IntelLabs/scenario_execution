# Scenario Execution Library for PyBullet

The `scenario_execution_pybullet` package provides actions to interact with the [PyBullet simulator](https://pybullet.org/).

It provides the following scenario execution library:

- `pybullet.osc`: PyBullet specific actions to interact with the simulation


## Example

To run the example, first build the `scenario_execution_pybullet` package and its dependencies:

```bash
colcon build --packages-up-to scenario_execution_pybullet
```

Source the workspace:

```bash
source install/setup.bash
```

Now, run the following command to launch the scenario:

```bash
./install/scenario_execution/lib/scenario_execution/scenario_execution libs/scenario_execution_pybullet/scenario_execution_pybullet/scenarios/example_pybullet.osc -s 0.00416666666 -t
```
