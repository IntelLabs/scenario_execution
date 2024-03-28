# Scenario Execution Base Package

The `scenario_execution` package is the base package for scenario execution. It provides functionalities like parsing, py-trees creation and execution.

It provides the following scenario execution libraries:

- `standard.osc`: The OpenSCENARIO 2 standard library. It is slightly modified to be in sync with the feature set of scenario execution. For convenience, numerical struct members are initialized with 0.
- `robotics.osc`: robotic-specific specifications
- `helper.osc`: helper actions
- `networking.osc`: actions related to networking

