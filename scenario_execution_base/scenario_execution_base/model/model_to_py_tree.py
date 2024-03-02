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

import py_trees
from py_trees.common import Access, Status
from pkg_resources import iter_entry_points

import inspect

from scenario_execution_base.model.types import EventReference, CoverDeclaration, ScenarioDeclaration, DoMember, WaitDirective, EmitDirective, BehaviorInvocation, EventCondition, EventDeclaration, RelationExpression, LogicalExpression, ElapsedExpression, PhysicalLiteral
from scenario_execution_base.model.model_base_visitor import ModelBaseVisitor
from scenario_execution_base.model.error import OSC2ParsingError


def create_py_tree(model, logger):
    model_to_py_tree = ModelToPyTree(logger)
    scenario = None
    try:
        scenario = model_to_py_tree.build(model)
    except OSC2ParsingError as e:
        logger.error(
            f'Error while creating tree:\nTraceback <line: {e.line}, column: {e.column}> in "{e.filename}":\n  -> {e.context}\n'
            f'{e.__class__.__name__}: {e.msg}'
        )
        return None
    return [scenario]


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

    def build(self, tree):
        if tree.find_children_of_type(CoverDeclaration):
            raise ValueError("Model still contains CoverageDeclarations.")
        behavior_builder = self.BehaviorInit(self.logger)
        behavior_builder.visit(tree)

        behavior_tree = behavior_builder.get_behavior_tree()

        if behavior_tree:
            print(py_trees.display.ascii_tree(behavior_tree))

        return behavior_tree

    class BehaviorInit(ModelBaseVisitor):
        def __init__(self, logger) -> None:
            super().__init__()
            self.logger = logger
            self.root_behavior = None
            self.__cur_behavior = None

        def get_behavior_tree(self):
            return self.root_behavior

        def visit_scenario_declaration(self, node: ScenarioDeclaration):
            scenario_name = node.qualified_behavior_name
            if self.root_behavior:
                raise ValueError(
                    f"Could not add scenario {scenario_name}. Scenario {self.root_behavior.name} already defined.")

            self.root_behavior = py_trees.composites.Sequence(name=scenario_name)
            self.__cur_behavior = self.root_behavior

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
                self.__cur_behavior.add_child(TopicPublish(
                    name=f"emit {node.event_name}", key=f"/{self.root_behavior.name}/{node.event_name}", msg=True))
            else:
                qualified_name = node.event.get_qualified_name()
                self.__cur_behavior.add_child(TopicPublish(
                    name=f"emit {node.event_name}", key=qualified_name, msg=True))

        def visit_behavior_invocation(self, node: BehaviorInvocation):
            behavior_name = node.behavior.name

            final_args = node.get_resolved_value()

            available_plugins = []
            for entry_point in iter_entry_points(group='scenario_execution.action_plugins', name=None):
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

            if not issubclass(behavior_cls, py_trees.behaviour.Behaviour):
                raise OSC2ParsingError(
                    msg=f"Found plugin for '{behavior_name}', but it's not derived from py_trees.behaviour.Behaviour.",
                    context=node.get_ctx()
                )

            plugin_args = inspect.getfullargspec(behavior_cls.__init__).args
            plugin_args.remove("self")

            final_args["name"] = node.name

            if node.actor:
                final_args["associated_actor"] = node.actor.get_resolved_value()
                final_args["associated_actor"]["name"] = node.actor.name

            missing_args = []
            for element in plugin_args:
                if element not in final_args:
                    missing_args.append(element)
            if missing_args:
                raise OSC2ParsingError(
                    msg=f'Plugin {behavior_name} requires arguments that are not defined in osc. Missing: {missing_args}', context=node.get_ctx()
                )
            log_name = None
            if final_args["name"]:
                log_name = final_args["name"]
            else:
                log_name = type(node).__name__
            self.logger.info(
                f"Instantiate action '{log_name}', plugin '{behavior_name}' with:\nArguments: {final_args}")
            try:
                instance = behavior_cls(**final_args)
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
            child = node.find_first_child_of_type(PhysicalLiteral)
            if child is None:
                raise OSC2ParsingError(
                    msg=f'Elapsed expression currently only supports PhysicalLiteral.', context=node.get_ctx())
            return child.get_resolved_value()

        def visit_event_declaration(self, node: EventDeclaration):
            if node.name in ['start', 'end', 'fail']:
                raise OSC2ParsingError(
                    msg=f'EventDeclaration uses reserved name {node.name}.', context=node.get_ctx()
                )
            else:
                qualified_name = node.get_qualified_name()
                client = self.__cur_behavior.attach_blackboard_client()
                client.register_key(qualified_name, access=Access.WRITE)
