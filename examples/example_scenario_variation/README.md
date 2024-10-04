# Example Scenario Variation

To run the Example Scenario Variation with scenario, first build Packages `scenario_execution` and `scenario_execution_coverage`:

```bash
colcon build --packages-up-to scenario_execution && colcon build --packages-up-to scenario_execution_coverage
```

Source the workspace:

```bash
source install/setup.bash
```

Now, run the following commands one by one to launch the scenario:

First

```bash
scenario_variation examples/example_scenario_variation/example_scenario_variation.osc
```
Second

```bash
scenario_batch_execution -i out -o scenario_output -- ros2 launch scenario_execution_ros scenario_launch.py scenario:={SCENARIO} output_dir:={OUTPUT_DIR}
```

For a more detailed understanding of the code structure and scenario implementation please refer to the [tutorial documentation](https://intellabs.github.io/scenario_execution/tutorials.html).
