# Copyright (C) 2024 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing,
# software distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions
# and limitations under the License.
#
# SPDX-License-Identifier: Apache-2.0

import copy
import py_trees
from py_trees.common import Access, Status
from pkg_resources import iter_entry_points

import inspect

from scenario_execution.model.types import ActionDeclaration, EventReference, FunctionApplicationExpression, ScenarioDeclaration, DoMember, WaitDirective, EmitDirective, BehaviorInvocation, EventCondition, EventDeclaration, RelationExpression, LogicalExpression, ElapsedExpression, PhysicalLiteral, ModifierDeclaration
from scenario_execution.model.model_base_visitor import ModelBaseVisitor
from scenario_execution.model.error import OSC2ParsingError
from scenario_execution.actions.base_action import BaseAction


def create_py_tree(model, tree, logger, log_tree):
    model_to_py_tree = ModelToPyTree(logger)
    try:
        model_to_py_tree.build(model, tree, log_tree)
    except OSC2ParsingError as e:
        raise ValueError(
            f'Error while creating py-tree:\nTraceback <line: {e.line}, column: {e.column}> in "{e.filename}":\n  -> {e.context}\n{e.__class__.__name__}: {e.msg}') from e


class TopicEquals(py_trees.behaviour.Behaviour):
    """
    Class to listen to a topic in Blackboard and check if it equals the defined message

    Args:
        key [str]: topic to listen to
        msg [str]: target message to match
        namespace [str]: namespace of the key
    """

    def __init__(self, key: str, msg: str, namespace: str = None):
        super().__init__(self.__class__.__name__)

        self.namespace = namespace
        self.key = key
        self.msg = msg

        self.client = self.attach_blackboard_client(namespace=self.namespace)
        self.client.register_key(self.key, access=Access.READ)

    def update(self):
        """
        Check the message on the topic equals the target message
        """
        msg_on_blackboard = self.client.get(self.key)
        if msg_on_blackboard == self.msg:
            return Status.SUCCESS
        return Status.RUNNING


class TopicPublish(py_trees.behaviour.Behaviour):
    """
    Class to publish a message to a topic

    Args:
        key [str]: topic to publish on
        msg [str]: message to publish on that topic
        namespace [str]: namespace of the key
    """

    def __init__(self, name: "TopicPublish", key: str, msg: str, namespace: str = None):
        super().__init__(name)

        self.namespace = namespace
        self.key = key
        self.msg = msg

        self.client = self.attach_blackboard_client(namespace=self.namespace)
        self.client.register_key(self.key, access=Access.WRITE)

    def setup(self, **kwargs):
        """
        Setup empty topic on blackboard

        This is to prevent the "Reader" from reading before the topic exists.
        """
        self.client.set(self.key, '')

    def update(self):
        """
        publish the message to topic
        """
        self.client.set(self.key, self.msg)
        return Status.SUCCESS


class ModelToPyTree(object):

    def __init__(self, logger):
        self.logger = logger

    def build(self, model, tree, log_tree):

        self.blackboard = tree.attach_blackboard_client(name="ModelToPyTree")
        behavior_builder = self.BehaviorInit(self.logger, tree)
        behavior_builder.visit(model)

        if log_tree:
            print(py_trees.display.ascii_tree(tree))

    class BehaviorInit(ModelBaseVisitor):
        def __init__(self, logger, tree) -> None:
            super().__init__()
            self.logger = logger
            self.__cur_behavior = tree

        def visit_scenario_declaration(self, node: ScenarioDeclaration):
            scenario_name = node.qualified_behavior_name

            self.__cur_behavior.name = scenario_name

            self.blackboard = self.__cur_behavior.attach_blackboard_client(
                name="ModelToPyTree",
                namespace=scenario_name)

            super().visit_scenario_declaration(node)

        def visit_do_member(self, node: DoMember):
            composition_operator = node.composition_operator
            if composition_operator == "serial":
                behavior = py_trees.composites.Sequence(name=node.name)
            elif composition_operator == "parallel":
                behavior = py_trees.composites.Parallel(name=node.name, policy=py_trees.common.ParallelPolicy.SuccessOnAll())
            elif composition_operator == "one_of":
                behavior = py_trees.composites.Parallel(name=node.name, policy=py_trees.common.ParallelPolicy.SuccessOnOne())
            else:
                raise NotImplementedError(f"scenario operator {composition_operator} not yet supported.")

            parent = self.__cur_behavior
            self.__cur_behavior.add_child(behavior)
            self.__cur_behavior = behavior
            self.visit_children(node)
            self.__cur_behavior = parent

        def visit_wait_directive(self, node: WaitDirective):
            child = node.get_only_child()

            if isinstance(child, (EventCondition, EventReference)):
                behavior = self.visit(child)

                self.__cur_behavior.add_child(behavior)
            else:
                raise OSC2ParsingError(msg="Invalid wait directive.", context=node.get_ctx())

        def visit_emit_directive(self, node: EmitDirective):
            if node.event_name in ['start', 'end', 'fail']:
                scenario_elem = node
                while scenario_elem is not None and not isinstance(scenario_elem, ScenarioDeclaration):
                    scenario_elem = scenario_elem.get_parent()
                self.__cur_behavior.add_child(TopicPublish(
                    name=f"emit {node.event_name}", key=f"/{scenario_elem.name}/{node.event_name}", msg=True))
            else:
                qualified_name = node.event.get_qualified_name()
                self.__cur_behavior.add_child(TopicPublish(
                    name=f"emit {node.event_name}", key=qualified_name, msg=True))

        def compare_method_arguments(self, method, expected_args, behavior_name, node):
            method_args = inspect.getfullargspec(method).args

            if "self" not in method_args:
                raise OSC2ParsingError(
                    msg=f'Plugin {behavior_name} {method.__name__} method is missing argument "self".', context=node.get_ctx())

            unknown_args = []
            missing_args = copy.copy(expected_args)
            for element in method_args:
                if element not in expected_args:
                    unknown_args.append(element)
                else:
                    missing_args.remove(element)
            error_string = ""
            if missing_args:
                error_string += "missing: " + ", ".join(missing_args)
            if unknown_args:
                if error_string:
                    error_string += ", "
                error_string += "unknown: " + ", ".join(unknown_args)
            return method_args, error_string

        def visit_behavior_invocation(self, node: BehaviorInvocation):
            if isinstance(node.behavior, ModifierDeclaration):
                available_modifiers = ["repeat"]
                if node.behavior.name not in available_modifiers:
                    raise OSC2ParsingError(
                        msg=f'Unknown modifier "{node.behavior.name}". Available modifiers {available_modifiers}.', context=node.get_ctx())
                if node.behavior.name == "repeat":
                    if isinstance(self.__cur_behavior, py_trees.composites.Composite):
                        instance = py_trees.decorators.Timeout(name="Repeat", child=py_trees.behaviours.Success(name="Have a Beer!"))#, num_success=3)
                    else:
                        raise OSC2ParsingError(msg=f'Modifier "{node.behavior.name}" currently only supported for composite node (i.e. one_of, parallel, serial).', context=node.get_ctx())
                else:
                    raise OSC2ParsingError(msg=f'Unknown modifier "{node.behavior.name}".', context=node.get_ctx())
            elif isinstance(node.behavior, ActionDeclaration):
                behavior_name = node.behavior.name
                available_plugins = []
                for entry_point in iter_entry_points(group='scenario_execution.actions', name=None):
                    # self.logger.debug(f'entry_point.name is {entry_point.name}')
                    if entry_point.name == behavior_name:
                        available_plugins.append(entry_point)
                if not available_plugins:
                    raise OSC2ParsingError(
                        msg=f'No plugins found for action "{behavior_name}".',
                        context=node.get_ctx()
                    )
                if len(available_plugins) > 1:
                    self.logger.error(f'More than one plugin is found for "{behavior_name}".')
                    for available_plugin in available_plugins:
                        self.logger.error(
                            f'Found available plugin for "{behavior_name}" '
                            f'in module "{available_plugin.module_name}".')
                    raise OSC2ParsingError(
                        msg=f'More than one plugin is found for "{behavior_name}".',
                        context=node.get_ctx()
                    )
                behavior_cls = available_plugins[0].load()

                if not issubclass(behavior_cls, BaseAction):
                    raise OSC2ParsingError(
                        msg=f"Found plugin for '{behavior_name}', but it's not derived from BaseAction.",
                        context=node.get_ctx()
                    )

                expected_args = ["self"]
                if node.actor:
                    expected_args.append("associated_actor")
                expected_args += node.get_parameter_names()

                # check plugin constructor
                init_method = getattr(behavior_cls, "__init__", None)
                init_args = None
                if init_method is not None:
                    # if __init__() is defined, check parameters. Allowed:
                    # - __init__(self)
                    # - __init__(self, resolve_variable_reference_arguments_in_execute)
                    # - __init__(self, <all-osc-defined-args>)
                    init_args, error_string = self.compare_method_arguments(init_method, expected_args, behavior_name, node)
                    if init_args != ["self"] and \
                            init_args != ["self", "resolve_variable_reference_arguments_in_execute"] and \
                            set(init_args) != set(expected_args):
                        raise OSC2ParsingError(
                            msg=f'Plugin {behavior_name}: __init__() either only has "self" argument or all arguments defined in osc. {error_string}\n'
                                f'expected definition with all arguments: {expected_args}', context=node.get_ctx()
                        )
                execute_method = getattr(behavior_cls, "execute", None)
                if execute_method is not None:
                    _, error_string = self.compare_method_arguments(execute_method, expected_args, behavior_name, node)
                    if error_string:
                        raise OSC2ParsingError(
                            msg=f'Plugin {behavior_name}: execute() arguments differ from osc-definition: {error_string}.', context=node.get_ctx()
                        )

                # initialize plugin instance
                action_name = node.name
                if not action_name:
                    action_name = behavior_name
                self.logger.debug(
                    f"Instantiate action '{action_name}', plugin '{behavior_name}'. with:\nExpected execute() arguments: {expected_args}")
                try:
                    if init_args is not None and init_args != ['self'] and init_args != ['self', 'resolve_variable_reference_arguments_in_execute']:
                        final_args = node.get_resolved_value(self.blackboard)

                        if node.actor:
                            final_args["associated_actor"] = node.actor.get_resolved_value(self.blackboard)
                            final_args["associated_actor"]["name"] = node.actor.name

                        instance = behavior_cls(**final_args)
                    else:
                        instance = behavior_cls()
                    instance._set_name_and_model(action_name, node)  # pylint: disable=protected-access
                except Exception as e:
                    raise OSC2ParsingError(msg=f'Error while initializing plugin {behavior_name}: {e}', context=node.get_ctx()) from e
                self.__cur_behavior.add_child(instance)

        def visit_event_reference(self, node: EventReference):
            event = node.resolve(node.event_path)
            name = event.get_qualified_name()
            return TopicEquals(key=name, msg=True)

        def visit_event_condition(self, node: EventCondition):
            expression = ""
            for child in node.get_children():
                if isinstance(child, RelationExpression):
                    raise NotImplementedError()
                elif isinstance(child, LogicalExpression):
                    raise NotImplementedError()
                elif isinstance(child, ElapsedExpression):
                    elapsed_condition = self.visit_elapsed_expression(child)
                    expression = py_trees.timers.Timer(
                        name=f"wait {elapsed_condition}s", duration=float(elapsed_condition))
                else:
                    raise OSC2ParsingError(
                        msg=f'Invalid event condition {child}', context=node.get_ctx())
            return expression

        def visit_elapsed_expression(self, node: ElapsedExpression):
            elem = node.find_first_child_of_type(PhysicalLiteral)
            if not elem:
                elem = node.find_first_child_of_type(FunctionApplicationExpression)

            if not elem:
                raise OSC2ParsingError(
                    msg=f'Elapsed expression currently only supports PhysicalLiteral and FunctionApplicationExpression.', context=node.get_ctx())

            return elem.get_resolved_value()

        def visit_event_declaration(self, node: EventDeclaration):
            if node.name in ['start', 'end', 'fail']:
                raise OSC2ParsingError(
                    msg=f'EventDeclaration uses reserved name {node.name}.', context=node.get_ctx()
                )
            else:
                qualified_name = node.get_qualified_name()
                client = self.__cur_behavior.attach_blackboard_client()
                client.register_key(qualified_name, access=Access.WRITE)
                setattr(client, qualified_name, None)
