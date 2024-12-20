# Scenario Execution

The `scenario_execution` package is the core package of Scenario Execution for Robotics. It provides functionalities such as parsing, py-trees creation and execution.

Furthermore, it provides the following scenario execution libraries:

- `standard.osc`: The OpenSCENARIO DSL standard library. It is slightly modified to be in sync with the feature set of scenario execution and imports `standard_base.osc`.
- `standard_base.osc`: The base parts of the OpenSCENARIO DSL standard library such as units and basic structs. For convenience, numerical struct members are initialized with 0.
- `robotics.osc`: robotic-specific specifications.
- `helper.osc`: helper actions such as logging or running external processes.

