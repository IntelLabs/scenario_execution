# Example PyBullet

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
./install/scenario_execution/lib/scenario_execution/scenario_execution  examples/example_pybullet/example_pybullet.osc -s 0.010416666
```
