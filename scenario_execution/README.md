# Scenario Execution

The `scenario_execution` package is the ROS2 middleware implementation of the scenario execution. It uses the `py_trees_ros` packages as the `py_trees`'s implementation for ROS2.

## ROSScenarioExecution

The class `ROSScenarioExecution` overrides the `setup_behaviour_tree()` and the `run()` method of the `ScenarioExecution`. It uses `py_trees_ros.trees.BehaviourTree` as behaviour tree. In the `run()` method, it uses `py_trees_ros.trees.BehaviourTree.tick_tock()` method to tick the behavior tree.

`py_trees_ros.trees.BehaviourTree` has a ROS2 node handle. Its ticking uses the ROS2 timer (`py_trees.trees.BehaviourTree` uses Python `time`). Therefore, it is intrinsically in sync with ROS2.

## Logger

The logger is adapted to use the ROS logger in [logging_ros.py](../scenario_execution/scenario_execution/logging_ros.py).

## Behaviors

### NavToPoseAction

Behavior to navigate to a pose.

### RosLogAction

Behavior to log using ROS2 logger.

### ROSTopicEquals

Behavior to check if the message on ROS topic equals to the target message.

### ROSTopicPublish

Behavior to publish a message on a ROS topic.

## Action Plugins

### NavToPose

Action plugin to navigate to a pose.

### RosLog

Action plugin to log a msg in ROS2.

### TfCloseTo

Action plugin to check if robot is close to a reference point. Might need additional functionality if used with simulation (e.g. [GazeboTfPublisher](../scenario_execution_gazebo/README.md###gazebo-ground-truth-publisher) to publish the robot ground truth position).
