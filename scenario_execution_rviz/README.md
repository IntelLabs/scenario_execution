# Scenario Execution Rviz

The `scenario_execution_rviz` package provides [rviz](https://github.com/ros2/rviz) plugins for visualizing and controlling the scenario when working with [ROS 2](https://docs.ros.org/en/rolling/index.html).

## Treeview Panel

The Treeview panel shows the behavior tree of the running scenario. The panel is accessible under Panels :arrow_right: Add new panel :arrow_right: scenario_execution_rviz :arrow_right: TreeView.

![tree_example.png](../docs/images/tree_example.png)

### Known Issues

The Treeview panel can not display the behavior tree if initialized while the robot navigation is already running.

### Icon Licence

The MIT License (MIT)
Copyright © 2019-2020 css.gg
