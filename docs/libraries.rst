Libraries
=========

Beside ``osc.standard`` provided by OpenSCENARIO 2 (which we divide into ``osc.standard`` and ``osc.standard_base``), multiple libraries are provided with scenario execution.

- ``osc.helpers`` Helpers Library (provided with :repo_link:`scenario_execution`)
- ``osc.robotics``: Robotics Library (provided with :repo_link:`scenario_execution`)
- ``osc.ros``: ROS Library (provided with :repo_link:`scenario_execution_ros`)
- ``osc.gazebo``: Gazebo Library (provided with :repo_link:`scenario_execution_gazebo`)

Additional features can be implemented by defining your own library.


Gazebo
------

The library contains actions to interact with the `Gazebo Simulation <https://gazebosim.org/>`_. Import it with ``import osc.gazebo``. It's provided by the package :repo_link:`scenario_execution_gazebo`.

Actions
^^^^^^^

``actor_exists()``
""""""""""""""""""

Waits for an actor to exist within simulation.

- ``entity_name``: Entity name within simulation
- ``world_name: string``: Gazebo world name (default: ``default``)

``osc_object.delete()``
"""""""""""""""""""""""

Delete an object from the simulation.

- ``entity_name``: Entity name within simulation
- ``world_name: string``: Gazebo world name (default: ``default``)

``osc_object.relative_spawn()``
"""""""""""""""""""""""""""""""

Spawn an actor relative to a given ``frame_id`` within simulation (at a specified ``distance`` in front of ``frame_id``).

- ``frame_id``: The frame Id to spawn the actor relative to. (default: ``base_link``)
- ``parent_frame_id``: The parent frame ID against which movement is evaluated. (default: ``map``)
- ``distance``: distance value relative to the frame_id at which to spawn the new actor (default: 1.0)
- ``world_name: string``: Gazebo world name (default: ``default``)
- ``model: string``: Model definition
- ``xacro_arguments: string``: (optional) Comma-separated list of argument key:=value pairs

``osc_object.spawn()``
""""""""""""""""""""""

Spawn an actor within simulation.

- ``spawn_pose: pose_3d``: Pose of the spawned actor.
- ``world_name: string``: Gazebo world name (default: ``default``)
- ``model: string``: Model definition
- ``xacro_arguments: string``: (optional) Comma-separated list of argument key:=value pairs

.. note::

    The model definition can be specified in different formats:

    - ``file://<path-to-model>``: Local path to model file
    - ``model://<path-to-model>``: Path relative to available model search paths
    - ``<package-name>://<path-to-model>``: Path relative to an available package (e.g. :repo_link:`simulation/gazebo/test_scenario_execution_gazebo/scenarios/test_spawn_exists_delete.osc`)
    - ``https:://fuel``: Model from `fuel.gazebosim.org <https://app.gazebosim.org/>`__ (e.g. ``https://fuel.gazebosim.org/1.0/OpenRobotics/models/Beer``)

    If the file ending is ``.xacro`` the model is forwarded to `xacro <https://wiki.ros.org/xacro>`__ before getting spawned.

``wait_for_sim()``
""""""""""""""""""
Wait for simulation to become active (checks for simulation clock).

- ``world_name: string``: Gazebo world name (default: ``default``)
- ``timeout: time``:  time to wait for the simulation. return failure afterwards. (default: ``60s``)


Helpers
-------

The library contains basic helper methods. Import it with ``import osc.helpers``.

Actions
^^^^^^^

``log()``
"""""""""

For debugging purposes, log a string using the available log mechanism.

- ``msg: string``: String to log

``run_process()``
""""""""""""""""""""""""""

Run a process. Reports `running` while the process has not finished.

If ``wait_for_shutdown`` is ``false`` and the process is still running on scenario shutdown, ``shutdown_signal`` is sent. If the process does not shutdown within shutdown_timeout, ``signal.sigkill`` is sent.

- ``command: string``: Command to execute
- ``wait_for_shutdown: bool``:  Wait for the process to be finished. If false, the action immediately finishes (default: ``true``)
- ``shutdown_signal: signal``: (Only used if ``wait_for_shutdown`` is ``false``) Signal that is sent if a process is still running on scenario shutdown (default: ``signal!sigterm``)
- ``shutdown_timeout: time``: (Only used if ``wait_for_shutdown`` is ``false``) time to wait between ``shutdown_signal`` and SIGKILL getting sent, if process is still running on scenario shutdown (default: ``10s``)

OS
--

The library contains actions to interact with the operating system. Import it with ``import osc.os``. It is provided by the package :repo_link:`libs/scenario_execution_os`.

Actions
^^^^^^^

``check_file_exists()``
"""""""""""""""""""""""

Report success if a file exists.

- ``file_name: string``: File to check


``check_file_not_exists()``
"""""""""""""""""""""""""""

Report success if a file does not exist.

- ``file_name: string``: File to check


Robotics
--------

The library contains elements reusable in different robotic contexts. Import it with ``import osc.robotics``. It is provided by the package :repo_link:`scenario_execution`.

Actors
^^^^^^

``robot``
"""""""""
A general robot actor.


ROS
---

The library contains actions to interact with ROS nodes. Import it with ``import osc.ros``. It is provided by the package :repo_link:`scenario_execution_ros`.

Actors
^^^^^^

``differential_drive_robot``
""""""""""""""""""""""""""""
A differential drive robot actor inheriting from the more general ``robot`` actor

Actions
^^^^^^^

``action_call()``
"""""""""""""""""
Call a ROS action and wait for the result.

- ``action_name: string``: Name of the action to connect to
- ``action_type: string``: Class of the action type (e.g. ``example_interfaces.action.Fibonacci``)
- ``data: string``: Call content (e.g. ``{\"order\": 3}``)

``assert_lifecycle_state()``
""""""""""""""""""""""""""""
Checks for the state of a `lifecycle-managed <https://design.ros2.org/articles/node_lifecycle.html>`__ node.

- ``node_name: string``: Name of ``lifecycle-managed`` node.
- ``state_sequence: list of lifecycle_state``: # List of states that a node is expected to transition through. The last entry is the state that a node is expected to remain in. Allowed ``['unconfigured', 'inactive', 'active', 'finalized]`` (e.g. ``[lifecycle_state!inactive, lifecycle_state!active]``)
- ``allow_inital_state_skip: bool`` if true, enables skipping of states within the state_sequence. (default: ``false``)
- ``fail_on_finish: bool``: If false action success, if node is in different state. (default: ``true``)

``assert_tf_moving()``
""""""""""""""""""""""

Checks that a tf `frame_id` keeps moving in respect to a `parent_frame_id`. If there is no movement within `timeout` the action ends, depending on `fail_on_finish`, either with success or failure. Speeds below `threshold_translation` and `threshold_rotation` are discarded. By default the action waits for the first transform to get available before starting the timeout timer. This can be changed by setting `wait_for_first_transform` to `false`. If the tf topics are not available on `/tf` and `/tf_static` you can specify a namespace by setting `tf_topic_namespace`.

- ``frame_id``: The frame Id to check for movement.
- ``parent_frame_id``: The parent frame ID against which movement is evaluated. (default: ``map``)
- ``timeout``: Timeout without movement.
- ``threshold_translation``: Translation speed below this threshold is skipped. (default: ``0.01mps``)
- ``threshold_rotation``: Rotational speed below this threshold is skipped. (default: ``0.01radps``)
- ``fail_on_finish``: If false, the action should success if no movement. (default: ``true``)
- ``wait_for_first_transform``: If true, start measuring only after first message is received. (default: ``true``)
- ``tf_topic_namespace``: namespace of `tf` and `tf_static` topic. (default: ``''``)
- ``use_sim_time``: In simulation, we need to look up the transform at a different time as the scenario execution node is not allowed to use the sim time (default: ``false``)

``assert_topic_latency()``
""""""""""""""""""""""""""

Check the latency of the specified topic (in system time). If the check with `comparison_operator` gets true, the action ends, depending on `fail_on_finish`, either with success or failure.

- ``topic_name: string``:  Topic name to wait for message
- ``latency: time``: The time to compare.
- ``comparison_operator: comparison_operator``: operator to compare latency time. (default: ``le``)
- ``fail_on_finish: bool``: If false action success, if comparison is true. (default: ``true``)
- ``rolling_average_count: int``: check for the latency over the x elements. (default: ``1``)
- ``wait_for_first_message: bool``: if true, start measuring only after first message is received. (default: ``true``)
- ``topic_type: string``: Class of message type, only required when 'wait_for_first_message' is set to false (e.g. ``std_msgs.msg.String``)


``check_data()``
""""""""""""""""
Wait for a topic message, compare a message field against a specific value.

In the background, this action uses `check_data() <https://py-trees-ros.readthedocs.io/en/devel/modules.html#py_trees_ros.subscribers.CheckData>`__ from `py_trees_ros <https://github.com/splintered-reality/py_trees_ros>`__.

- ``topic_name: string``: Name of the topic to connect to
- ``topic_type: string``: Class of the message type (e.g. ``std_msgs.msg.String``)
- ``qos_profile: qos_preset_profiles``: QoS Preset Profile for the subscriber (default: ``qos_preset_profiles!system_default``)
- ``variable_name: string``: Name of the variable to check
- ``expected_value: string``: Expected value of the variable
- ``comparison_operator: comparison_operator``: The comparison operator to apply (default: ``comparison_operator!eq``)
- ``fail_if_no_data: bool``: return failure if there is no data yet (default: ``false``)
- ``fail_if_bad_comparison: bool``: return failure if comparison failed (default: ``true``)
- ``clearing_policy: clearing_policy``: When to clear the data (default: ``clearing_policy!on_initialise``)

``differential_drive_robot.init_nav2()``
""""""""""""""""""""""""""""""""""""""""

Initialize nav2.

- ``initial_pose: pose_3d``: The initial pose to set during initialization
- ``base_frame_id: string``: Base Frame ID (default: ``base_link``)
- ``use_initial_pose: bool``: If false, no initial_pose is needed (useful when using slam instead of amcl for localization) (default: ``true``)
- ``namespace_override: string``: If set, it's used as namespace (instead of the associated actor's namespace)
- ``wait_for_initial_pose: bool``: If true the initial pose needs to be set externally (e.g. manually through rviz)(default: ``false``)

``differential_drive_robot.nav_through_poses()``
""""""""""""""""""""""""""""""""""""""""""""""""

Use nav2 to navigate through poses.

- ``goal_pose: string``: Goal poses to navigate to (format: ``x1,y1,yaw1;x2,y2,yaw2;...``)
- ``namespace_override: string``: If set, it's used as namespace (instead of the associated actor's namespace)
- ``monitor_progress: bool``: If yes, the action returns after the goal is reached or on failure. If no, the action returns after request. (default: ``true``)

``differential_drive_robot.nav_to_pose()``
""""""""""""""""""""""""""""""""""""""""""
Use nav2 to navigate to goal pose.

- ``goal_pose: pose_3d``: Goal pose to navigate to
- ``namespace_override: string``: If set, it's used as namespace (instead of the associated actor's namespace)
- ``action_topic: string``: Action name (default: ``navigate_to_pose``)
- ``monitor_progress: bool``: If yes, the action returns after the goal is reached or on failure. If no, the action returns after request. (default: ``true``)

``differential_drive_robot.odometry_distance_traveled()``
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""

Wait until a defined distance was traveled, based on odometry.

- ``distance: length``: Traveled distance at which the action succeeds.
- ``namespace_override: string``: if set, it's used as namespace (instead of the associated actor's namespace)

``differential_drive_robot.tf_close_to()``
""""""""""""""""""""""""""""""""""""""""""

Wait until a TF frame is close to a defined reference point.

- ``namespace_override: string``: if set, it's used as namespace (instead of the associated actor's namespace)
- ``reference_point: position_3d``: Reference point to measure to distance to (z is not considered)
- ``threshold: length``: Distance at which the action succeeds.
- ``sim: bool``: In simulation, we need to look up the transform map --> base_link at a different time as the scenario execution node is not allowed to use the sim time (default: ``false``)
- ``robot_frame_id: string``: Defines the TF frame id of the robot (default: ``base_link``)

``log_check()``
"""""""""""""""
Wait for specific output in ROS log (i.e. `/rosout` topic). If any of the entries within ``values`` the action succeeds.

- ``module_name: string``: if specified, a matching message must also match the module name (default: empty)
- ``values: list of string``: list of strings (in python syntax, e.g. "[\'foo\', \'bar\']")

``record_bag()``
""""""""""""""""

Record a ROS bag, stored in directory ``output_dir`` defined by command-line parameter (default: '.')

- ``topics: list of string``: List of topics to capture
- ``timestamp_suffix: bool``: Add a timestamp suffix to output directory name (default: ``true``)
- ``hidden_topics: bool``: Whether to record hidden topics (default: ``false``)
- ``storage: string``: Storage type to use (empty string: use ROS bag record default)

``ros_launch()``
""""""""""""""""

Execute a ROS launch file.

- ``package_name: string``: Package that contains the launch file
- ``launch_file: string``: Launch file name
- ``arguments: list of ros_argument``: ROS arguments (get forwarded as key:=value pairs)
- ``wait_for_shutdown: bool``: If true, the action waits until the execution is finished (default: ``true``)
- ``shutdown_timeout: time``: (Only used ``if wait_for_shutdown`` is ``false``) Time to wait between ``SIGINT`` and ``SIGKILL`` getting sent, if process is still running on scenario shutdown (default: ``10s``)

``service_call()``
""""""""""""""""""

Call a ROS service and wait for the reply.

- ``service_name: string``: Name of the service to connect to
- ``service_type: string``: Class of the message type (e.g. ``std_srvs.msg.Empty``)
- ``data: string``: Service call content

``set_node_parameter()``
""""""""""""""""""""""""

Set a parameter of a node.

- ``node_name: string``: Name of the node
- ``parameter_name: string``: Name of the parameter
- ``parameter_value: string``: Value of the parameter

``topic_publish()``
"""""""""""""""""""

Publish a message on a topic.

- ``topic_name: string``: Name of the topic to publish to
- ``topic_type: string``: Class of the message type (e.g. ``std_msgs.msg.String``)
- ``qos_profile: qos_preset_profiles``: QoS Preset Profile for the subscriber (default: ``qos_preset_profiles!system_default``)
- ``value: string``: Value to publish

``wait_for_data()``
"""""""""""""""""""

Wait for a specific data on a ROS topic.

In the background, this action uses `wait_for_data() <https://py-trees-ros.readthedocs.io/en/devel/modules.html#py_trees_ros.subscribers.WaitForData>`__ from `py_trees_ros <https://github.com/splintered-reality/py_trees_ros>`__.

- ``topic_name: string``: Name of the topic to connect to
- ``topic_type: string``: Class of the message type (e.g. ``std_msgs.msg.String``)
- ``qos_profile: qos_preset_profiles``: QoS Preset Profile for the subscriber (default: ``qos_preset_profiles!system_default``)
- ``clearing_policy: clearing_policy``: When to clear the data (default: ``clearing_policy!on_initialise``)


``wait_for_topics()``
"""""""""""""""""""""

Wait for topics to get available (i.e. publisher gets available).

- ``topics: list of string``: List of topics to wait for
