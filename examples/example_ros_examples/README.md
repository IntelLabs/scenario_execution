# Example Test Publisher

Test the functionality of `examples_rclpy_*`with `scenario_execution_ros`.


```bash
sudo apt install ros-jazzy-examples-rclpy-*
colcon build --packages-up-to scenario_execution_ros
```

Source the workspace:

```bash
source install/setup.bash
```

Now, run the following command to launch the scenario:

```bash
ros2 run scenario_execution_ros scenario_execution_ros examples/example_test_publisher/test_publisher.osc
```
