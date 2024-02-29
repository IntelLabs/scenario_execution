# Scenario Execution Base Package

The "scenario_execution_base" package is the base package for scenario execution. It provides functionalities like parsing and base classes.

The "scenario_execution_base" package has following characteristics:

- middleware-, simulation- and robot-agnostic
- using [py_trees](https://github.com/splintered-reality/py_trees)
- using [OpenSCENARIO V2.0.0](https://asam-ev.github.io/public_release_candidate/asam-openscenario/2.0.0/welcome.html) as input format

## OpenScenario 2.0 Parsing

The osc files are parsed using [py-osc2](https://github.com/PMSFIT/py-osc2) package. The `ScenarioVisitor` from the [osc2_parser.py](../scenario_execution_base/scenario_execution_base/osc2_utils/osc2_parser.py) will visit the parsed tree and pass the parsed scenarios as the root of the `py_trees.trees.BehaviorTree`, the parsed events, and the parsing errors to scenario execution.
Scenario execution will then start to execute the scenarios one by one.

## OpenScenario 2.0 Types

The syntax of OpenScenario 2.0 involves many types. "Under the cover", the osc2 types are all implemented as Python types, see [osc2_type.py](../scenario_execution/scenario_execution/osc2_type.py) for more details. Here are some implementation details about the osc2 type implementation.

### Identifiers and Namespaces

To enable the identifier of OpenSCENARIO V2.0.0 in the parser, namespaces are introduced. In OpenSCENARIO 2, The names of physical types and units, enum types and their members, structured types and their members are all identifiers and they should be unique and do not allow name conflicts. Morever, they should be able to be referenced, e.g., in expression and type declarator.

There are three levels of namespace in the parser. The global namespace is the first level. It contains global identifiers. The namespace of structured types is the second level. It constains identifiers within the namespace of the structured types. The third level is the parameters of the structured types.
In the namespace of the parameter, the namespace is used to evaluate the constraints if the parameter value is a structured type.

![osc2_namespace.png](graphs/osc2_namespace.png)

The namespace is implemented as a Python class `Namespace` in the [osc2_type. py](../scenario_execution_base/scenario_execution_base/osc2_utils/osc2_type.py). The Python class `Namespace` serves the function of the namespace stack, with the top namespace representing the current namespace. The methods `push()` and `pop()` are there to push and pop the top namespace.
The method `get()` will return the current namespace (the top of the stack) without popping it.

Under each namespace, there is a "identifiers". The identifiers are also implemented as a Python class `Identifiers` in the [osc2_type. py](../scenario_execution_base/scenario_execution_base/osc2_utils/osc2_type.py).
The `ScenarioVisitor` in [osc2_parser.py](../scenario_execution_base/scenario_execution_base/osc2_utils/osc2_parser.py) and `StucturedType` in [osc2_type. py](../scenario_execution_base/scenario_execution_base/osc2_utils/osc2_type.py) have an `Identifiers` as their attribute. It registered the identifiers in the current namespace.
The method `register()` registers the identifiers and checks if there's a naming conflict. The method `get_identifier()` return the Python object the identifier references to. The method `has_identifier()` checks if the identifier already exists in the namespace.

In `ScenarioVisitor` in [osc2_parser.py](../scenario_execution_base/scenario_execution_base/osc2_utils/osc2_parser.py), there is also a helper function `search_for_identifier()` to search all identifiers in the all levels of namespaces.

### Physical Types and Units

Physical types have physical qualities. They are defined with a name and a SI unit exponents. Different units with factor and offset of one physical type can be defined.

Each unit type has a `convert_to_si()` method. It will return the value in SI units.

The osc2_parser will parser value with units, e.g., `2m`, `5.0s`, as "physical literal". The "physical literal" is wrapped in a `PhysicalLiteral` type with a value and references to that phyiscal type and unit type. The `PhysicalLiteral` stores the value in its declared unit. If you want to convert the value in and/or from SI units, use the methods `convert_to_si()` and `convert_from_si()`.

### Enumeration Types

`enum` types link a member with an integer. The value of `enum` types can be assigned with both integer and member identifier. The members are stored as a dict under `EnumType().members`. The keys are integers and the values are the member identifiers.

When declaring an `enum` type, if no integers are given for a certain member, the next available integer will be linked to the member. If an integer is given, the enum type will first check if the integer is already taken. If so, it will raise a ValueError. If not, the integer will be assigned to that member.

The members of the `enum` are also registered as global identifiers. In order to circumvent the issues that different `enum` types have the same member names and cause identifier conflicts, the identifiers of the enum members are registered with strings and will be evaluated when it is assigned to a parameter of `enum` type.
In order to be able to compare the values of the `enum` types, the `enum_name!enum_member_name` is also registered as an identifier and can be referenced with `enum_value_reference` syntax of OpenSCENARIO 2.

### Structured Types

Structured types are an important part of OpenSCENARIO V2.0.0. They can be structs, actors, scenarios, and actions. All of them can have event, field (parameters and variables), method, constraint and coverage members.

In Python implementation, the `StructuredType` is a base class for `Struct`, `Actor`, `Scenario`, and `Action`. Different structured types in OpenSCENARIO 2 are implemented as objects of `Struct`, `Actor`, `Scenario`, and `Action`. In OpenSCENARIO2, the structured types can also inherit from parent structured types.
Therefore, in Python, the child structured types (a Python object) will deepcopy the parameters, variables, and methods of the parent structured types (a Python object).

Structs, actors, scenarios and actions all have different functionalities on their own and their implemenation details will be introduced in the following.
In the following, the capitalized `Struct`, `Actor`, `Scenario`, and `Action` refer to the Python implementation of the structured types, while the uncapitalized `struct`, `actor`, `scenario`, and `action` refer to the structured types in OpenSCENARIO V2.0.0.

#### Struct

According to OpenSCENARIO V2.0.0, the `struct` is a complex datatype to group multiple parameters, variables, methods and other base members of structured types. It does not offer any additional members beyond the base structured type members.

In Python, `Struct` is implemented as a pure inherit of the base class `StructuredType`.

#### Actor

Compared to structs, actors can perform behaviors (including actions and scenarios). Scenarios and actions can be associated with the instance of actors and access the parameters of the actor.

Within the `Actor` class, use `self.associated_behaviors` to access the associated actions and scenarios.

#### Scenario

Scenarios can have associated actor or can be standalone scenarios.

Standalone scenarios are registered in the `ScenarioVisitor` under `self.scenarios`. Standalone scenarios are also registered as an identifier in the global namespace. Scenarios with associated actor are registered under the actor's namespace as an associated behavior and an identifier in the actor's namespace.

<strong>Note: Only standalone scenarios are executed by scenario execution.</strong>

Compared to struct and actor, scenarios have two more members: modifiers and constraints. It also has behavior specifications (do_directive and on_directive).

According to OpenSCENARIO V2.0.0 [scenarios](https://asam-ev.github.io/public_release_candidate/asam-openscenario/2.0.0/language-reference/types.html#sec-lc-types-scenarios), the scenario can have one "do_directive" and multiple "on_directive". The py_trees structure is as shown.

![scenario_structure.png](graphs/scenario_structure.png)

The "on_directive" will wait on the specified event to be emitted. Then, it will continue to execute its members.

The "do_directive" is the main actions of the scenario and will be executed from the start.

The scenario root will be a `py_trees.composites.Parallel` and has children of the "do_directive" and all the "on_directive"s.

Call the method `get_root()` will return the root of the py_trees behavior tree.

"do_member"s of the "do_directive" can have an optional label name. The label name is registered as the an identifier in the scenario's namespace. Referencing to the label name will return the root of the subtree. The label name also serves as the name of the node in py_trees. If the label name is omitted, the name of the py_trees behaviour will be used as the name of the node automatically by py_trees.

#### Action

According to OpenSCENARIO V2.0.0, actions should be treated as atomic behavior descriptions and their internal structure should be opaque from user's prespective. Therefore, actions are ideal for plugin implementation.

Actions have the same attributes and methods as scenarios. It can also contain the same members as scenarios and behavior specifications.

Actions can also be linked to external code. This is done through scenario execution's plugin mechanism. The `plugin` parameter of actions are reserved for plugin interfaces. The `plugin` parameter should be of type `string` in OpenSCENARIO 2. If a valid plugin is provided for actions. The rest of the action description will be ignored (including do_diretive and on_directives).

For more information on plugin mechanism of actions, please refer to [Action Plugins](#actions-plugins).

### Structured Type Members

Structured types can also have various members. Specifications and implementation details will be explained in the following.

Events, fields, constraints, methods and coverages are common members of structured types. They can be members of structs, actors, scenarios and actions. Only scenarios and actions can contain modifiers.

#### Event

Events are members of structured types. Events are in general indicators for whether something has happened. They can be defined, emitted and referenced during the execution of scenarios.

The events are registered in the py_trees blackboard. The "emit_directive" will return a `scenario_execution.behaviors.TopicPublish` and publish `True` in the blackboard. The "wait_directive" if followed by a "event_reference" will return a `scenario_execution.behaviors.TopicEquals` and checks for that event in the blackboard while ticking.

According to OpenSCENARIO V2.0.0 [pre-defined events](https://asam-ev.github.io/public_release_candidate/asam-openscenario/2.0.0/language-reference/types.html#sec-lc-type-pre-defined-events), the event `start`, `end`, and `fail` are reserved for scenario state indication.Trying to declare them will result in errors.
Emitting the `end` event will stop the execution of the scenario and the scenario will be considered as executed successfully, while emitting the `fail` event will also stop the execution of the scenario and the scenario will be considered as failed.

The events are registered under `self.events` and registered on the blackboard as `$SCENARIO_NAME/$EVENT_NAME` (with scenario namespace). When referencing events, please use the full name `scenario_name.event_name` to indicate event.

#### Fields

Fields are members of the structured types. They can be a parameter or a variable. Parameters have fixed value by the end of the parsing process while variable can change during runtime.

Parameters can be primitive types (int, uint, float, string, and bool), enum types, physical types, structured types (struct and actor), and aggregate types. Each parameter has a declared type. It can be accessed under its scope through `self.declared_type`.

The actual values of the parameters can be accessed through `self.value`. There are in general two ways of assignment values to parameters. The first is through default value assignment during declaration. The second is through constraints. For more information on assigning values through constraints, please refer to [Section: Constraints](#constraints).

In Python, parameters are implemented as the object of `Parameter` from [osc2_type.py](../scenario_execution_base/scenario_execution_base/osc2_utils/osc2_type.py). The value of the parameter is implemented as a class property. The value of a parameter is governed by the constraints that the parameter has.
The value will first evaluate the "hard" constraint of the parameter and if no "hard" constraints are applied to the parameter.
The "default" constraint will be used. According to [OpenSCENARIO V2.0.0](https://www.asam.net/static_downloads/public/asam-openscenario/2.0.0/language-reference/types.html#sec-lc-types-constraint-strength), parameters can only have one "hard" constraint and its default constraint can be overridden by a "hard" constraint or another "default" constraint.
Type checks are applied to the parameter, when constraints are applied to the parameter. If the type of the value does not match the declared type of the parameter, errors will be raised.

The values of different declared types are different in order to accommodate the characteristics of different types. Primitive types parameters are saved as their actual value in Python primitive types. `int` and `uint` from osc2 are stored as `int` in Python with positive value constraint for `uint` (TODO).
`float` from osc2 is stored as `float` in Python. `string` from osc2 is stored as `str` in Python. `bool` from osc2 is stored as `bool` in Python. `enum` types are stored as integer in the parameter.
The `enum` types need to be assigned with identifiers of its members. Physical types are stored as an object of `PhysicalLiteral` in Python. Structured types are stored as a deepcopy of that type's instance. The usage of `deepcopy` is resulted from the deeply nested data structure of the structured types.
In order to be able to apply constraints to the structured types' members. Their members need to first be initialized.
It is possible to have multiple instances in osc2 syntax of one structured types. Therefore, the instance of the structured type in Python is deepcopied.

| Parameter type (in OSC2) | Python types ("type_declarator")        | Value                                   |
| ------------------------ | --------------------------------------- | --------------------------------------- |
| Pritimive types          |  class of `int`, `float`, `str`, `bool` | Object of `int`, `float`, `str`, `bool` |
| Enum                     | Object of `EnumType`                    | Object of `int`                         |
| Physical types           | Object of `PhysicalType`                | Object of `Physical`                    |
| Structured types         | Object of `StructuredType`              | Deepcopy of the object                  |
| Aggregate types          | Object of `AggregateType`               | Object of `AggregateType`               |

Variables are similar to parameters, but can change during the runtime. Because of the dynamic characteristics of the variables, they are currently not yet supported by scenario execution. The variables in the [standard library](../scenario_execution_base/scenario_execution_base/lib_osc/standard.osc) are currently changed to parameters so that the standard library can be successfully parsed.

#### Methods

Methods are members of structured types. They can be implemented internally in osc2 as an expression (TODO) or they can use external code. In the current parser, the feature of implementing the methods internally from expression is not supported.

Because the arguments of the methods may not be set by the declaration of the method, the methods are implemented as `Method` at the time of declaration and is evaluated when it's called.

Methods can be defined without implementation in parent types and can be overriden by methods in children types with `only` syntax. Therefore, in structured types, there are two methods implemented: `add_method()` and `override_method()`.

According to OpenSCENARIO V2.0.0, only methods without return types can be called by call directive. Therefore, methods without returned type can only use external code by using `external` syntax and it should return a `py_trees.behaviour.Behaviour` instance.

Methods can also be called as function application in the expression. These methods can return arbitrary types (not necessarily of `py_trees.behaviour.Behaviour`) that matches the declared return type. Checks are applied to methods with return types. If the return type does not matched the declared one, errors will be raised by the parser.

Positional arguments and named arguments can be specified when declaring the method. Arguments without default values in the argument list specification are considered as positional arguments, while arguments with default values are considered as named arguments.
The positional arguments and the named arguments from the argument list specification are then appended to the arguments that are already given in the method implementation and passed to external code.

The external code of the method implementation are specified in Python import syntax. For example, specification like `math.sqrt` will use the `sqrt()` function from the package `math` in Python. The external methods only support Python code.
To use code in other programming language, such as C++, the interface (`visitMethod_implementation` function from [osc2_parser.py](../scenario_execution_base/scenario_execution_base/osc2_utils/osc2_parser.py)) needs to be adapted.

#### Constraints

Constraints can be members of parameters and structured types. There are two types of constraints: `keep_constraint` and `remove_default`.

`keep_constraint` applies to assign values to the parameter. `keep_constraint` is implemented as a class `KeepConstraint` in Python in [osc2_type.py](../scenario_execution_base/scenario_execution_base/osc2_utils/osc2_type.py). `KeepConstraint`s are only applied to parameters, because even if the structured types need constraints, they are still constraining the their parameters.
The method `evaluate()` will apply the constraint to the field (parameter) and the method `remove()` will remove the applied constraint from the field (parameter).

`remove_default` constraints are currently not supported. However, they could be easily implemented with the method `remove()` from `KeepConstraint` class.

### Aggregate Types

Aggregate types are a list of types in OpenSCENARIO 2. They are subscriptable and their members should have the same type.

Aggregate types are implemented as Python class `AggregateType` in [osc2_type.py](../scenario_execution_base/scenario_execution_base/osc2_utils/osc2_type.py). They have a declared type for its members. The object of class `AggregateType` is subscritable and is used to access it members from expression, so that expressions, such as "aggregate_type[0]" can be parsed by the parser.

Aggregate Types also have methods: `size()`, `filter()`, `first_index()`, `count()`, `has()`, `map()`. They are currently not supported.
In order to support them, the `visitFunction_application_pe` method from [osc2_parser.py](../scenario_execution_base/scenario_execution_base/osc2_utils/osc2_parser.py) needs to adapted to recognize the methods names in order to call the corresponding methods from Python class `AggregateType`.

#### Coverages

Coverages still need implementation.

#### Modifiers

Modifiers still need implementation.

### Module API

The OpenSCENARIO V2.0.0 syntax provides several interfaces for using external code.

#### External methods

According to OpenSCENARIO V2.0.0 [external methods](https://asam-ev.github.io/public_release_candidate/asam-openscenario/2.0.0/language-reference/types.html#_external_methods), the methods can use external implementation.

The external methods can use external Python code. They can be declared in the OpenSCENARIO 2 with following syntax as an example:

```{{< mdl-disable "MD040" >}}
struct base_methods:
    # External methods that use python standard libraries
    def sqrt(value: float) -> float is external math.sqrt()

    # External methods that use scenario execution "external_methods" module
    def square(value: float) -> float is external scenario_execution.external_methods.power(2)
    def string_concat(string_1: string, string_2: string) -> string is external scenario_execution.external_methods.string_concat()
```

External methods can be called by `call_directive` and they should return an object of `py_trees.behaviour.Behaviour`. Then, it can be incorporated into the py_trees behaviour tree and executed by the scenario execution. External methods that return a py_trees behavior should not have declared return type in OpenSCENARIO 2.

External methods can also be called by function application syntax in OpenSCENARIO 2. They should have a return type. The external Python code will then be executed during parsing.

For more details of how the method is implemented, please refer to [Section: Method](#methods).

#### Actions Plugins

The action syntax is another interface for using external code. According to OpenSCENARIO V2.0.0 [actions](https://asam-ev.github.io/public_release_candidate/asam-openscenario/2.0.0/language-reference/types.html#_actions), actions are considered as atomic behavior of the scenarios and is opaque from the users' perspective.

The parameter "plugin" of actions is reserved for linking the external plugin packages to the scenario execution. The "plugin" parameter needs to be of type `string` in OpenSCENARIO 2. The "plugin" parameter of actions will link the action to external `ActionPlugin` classes. While in behavior invocation, the `ActionPlugin` classes will return a fully instantiated py_trees behavior for scenario execution.

To create a plugin package, first inherit from `scenario_execution.osc2_utils.osc2_type.ActionPlugin` . Then, override the `parse()` method to parse the necessary arguments from the action handle. After that, override the method `create_behavior()` to return the needed py_trees behavior.
The ActionPlugin class has an attribute `self.action_handle` for the corresponding action object in the parser. The actions parameters and action associated actor parameters can be accessed through the action handle.

For an example of an action plugin, please refer to [base_log.py](../scenario_execution/scenario_execution/action_plugins/ros_log.py).

There are already base action plugin classes for actions defined in the "amr" and "builtin" libraries.

After creating the action plugin, register the action plugin in the setup file as an entry point of `scenario_execution.action_plugins`, so that it could be automatically detected by the parser.

For an example, please refer to [setup.py](../scenario_execution/setup.py).

A tutorial for creating a action plugin, please refer to [Tutorial: Create action plugin](tutorials.md#create-action-plugin).

### Error Logging

### Errors

There are two errros implemented for scenario execution in [error.py](../scenario_execution_base/scenario_execution_base/osc2_utils/error.py): `OSC2ParsingError` and `ScenarioExecutionRuntimeError`.

The `OSC2ParsingError`s are raised by the `OSC2Parser` and `ScenarioVisitor`. The `OSC2ParsingError` has attributes for `msg`, `line`, `column` and `context`. The `OSC2ParsingError`s are catched by the parser and the information is presented to the user to show in which line and column in the OpenSCENARIO 2 file the errors occur.

The `ScenarioExecutionRuntimeError`s are raised by `ScenarioExecution` and should be raised by its middleware implementation. They are catched by the `ScenarioExecution` itself and presented to the user with error information.

## Base Classes

The package "scenario_execution_base" constains base classes for implementing the scenario execution with middleware.

### Base Scenario Execution

The `ScenarioExecution` Python class is the base of middleware adaption for scenario execution. It is responsible for setting up the `py_trees` behavior tree and the logger. Then, it starts to run by ticking the `py_trees`. It also monitors the events of the scenarios and stops the execution if the scenario ends or fails.

To adapt the scenario execution to other middleware, inherit from the class `ScenarioExecution` and override the method `setup_behaviour_tree()` and return the corresponding `py_trees` implementation of that middleware. If the `py_trees` implementation of the new middleware uses a different ticking mechanism, it might require to override the method `run()` and adapt the ticking.

### Base Behaviors

Behaviors inherit from the class `py_trees.behaviour.Behaviour` and are implemented with desired functions. They can be incorporated into a py_trees behavior tree and ticked in every cycle. For more details of how the py_trees works, please refer to [py_trees](https://py-trees.readthedocs.io/en/devel/).

There are three base behaviors in the packages. They are essential to the parsing and execution of the scenario execution.

#### BoolExpression

The behavior `BoolExpression` gets a boolean expression upon instantiation and return `py_trees.Status.SUCCESS` if the expression is true and `py_trees.Status.FAILURE` if the expression is false.

The `BoolExpression` is used in the parser for `wait_directive` that is followed by a `event_condition`. If the `bool_expression` of the `event_condition` returns true, then the `BoolExpression` receives true upon instantiation and vice versa.

#### TopicPublish

The behavior `TopicPublish` publishes a msg to a key in the `py_trees` blackboard. It is used by `emit_directive` in scenario execution and it will publish if the event is emitted. For more information of OpenSCENARIO 2 `event`, please refer to [event](#event).

#### TopicEquals

The behavior `TopicEquals` checks a key in the `py_trees` blackboard. It is used by `wait_directive` followed by a `event_reference`. It returns true if an event is emitted.

### Base Action Plugins

The "scenario_execution_base" package also include five base action plugins. They cannot be used "out-of-box". They need to be inherited and adapt the returned `py_trees` behaviors to the middleware. They are implemented to parse the parameters from the actions defined in the OpenSCENARIO 2 libraries.

#### BaseActorExists

Class for checking the existance of an actor within an environment (e.g. a simulation).

#### BaseCloseTo

Class for checking if a robot is close to a reference point.

#### BaseDeleteActor

Class to delete an actor in simulation.

#### BaseLog

Class for logging a message in scenario execution.

#### BaseSpawnActor

Class for spawning obstacle plugin in within a simulation environment.

## Import Mechanism and Core Libraries

### Import Mechanism

The parser allows the user to import the code from the another OpenSCENARIO 2 file. The parser parses the imported file where the `import` expression resides. Therefore, it is good practice to place all import expressions at the top of the file. The parser keeps track of the imported file and does not import the same file twice, in case there are nested import expression.

### Core Libraries

The `scenario_execution_base` comes with some core libraries in OpenSCENARIO 2 syntax. They can imported with the expression:

```{{< mdl-disable "MD040" >}}
import osc.library_name
```

Other user defined libraries need to be imported by giving the path to the import file as a string:

```{{< mdl-disable "MD040" >}}
import "$(PATH_TO_IMPORTED_FILE)"
```

#### Standard Library

This is a modified version of the [standard library](https://www.asam.net/static_downloads/public/asam-openscenario/2.0.0/domain-model/standard_library.html) from OpenSCNEARIO V2.0.0. The modifiers are commented out since they still need to be implemented. Some modifications and reordering of the classes apply to the modified standard library to accommodate the current parser.
They are marked in the [standard library file](../scenario_execution_base/scenario_execution_base/lib_osc/standard.osc).

#### AMR Library

The AMR library contains classes defined for [`amr_playground`](https://github.com/intel-innersource/applications.robotics.mobile.amr-playground) repository.

#### Builtin Library

The builtin library contains some classes defined for using scenario execution more conveniently, including `log` action, `base_methods`, etc.
