Libraries
=========

Beside ``osc.standard`` provided by OpenSCENARIO 2 (which we divide into ``osc.standard`` and ``osc.standard_base``), multiple libraries are provided with scenario execution.

.. list-table:: 
   :widths: 40 60
   :header-rows: 1
   :class: tight-table   
   
   * - Name
     - Description
   * - ``osc.helpers``
     - Helpers Library (provided with :repo_link:`scenario_execution`)
   * - ``osc.robotics``
     - Robotics Library (provided with :repo_link:`scenario_execution`)
   * - ``osc.ros``
     - ROS Library (provided with :repo_link:`scenario_execution_ros`)
   * - ``osc.gazebo``
     - Gazebo Library (provided with :repo_link:`scenario_execution_gazebo`)

Additional features can be implemented by defining your own library.


Gazebo
------

The library contains actions to interact with the `Gazebo Simulation <https://gazebosim.org/>`_. Import it with ``import osc.gazebo``. It's provided by the package :repo_link:`scenario_execution_gazebo`.

Actions
^^^^^^^

``actor_exists()``
""""""""""""""""""

Waits for an actor to exist within simulation.

.. list-table:: 
   :widths: 15 15 5 65
   :header-rows: 1
   :class: tight-table   
   
   * - Parameter
     - Type
     - Default
     - Description
   * - ``entity_name``
     - ``string``
     -
     - Entity name within simulation
   * - ``world_name``
     - ``string``
     - ``default``
     - Gazebo world name

``osc_object.delete()``
"""""""""""""""""""""""

Delete an object from the simulation.

.. list-table:: 
   :widths: 15 15 5 65
   :header-rows: 1
   :class: tight-table   
   
   * - Parameter
     - Type
     - Default
     - Description
   * - ``entity_name``
     - ``string``
     - 
     - Entity name within simulation
   * - ``world_name``
     - ``string``
     - ``default``
     - Gazebo world name

``osc_object.relative_spawn()``
"""""""""""""""""""""""""""""""

Spawn an actor relative to a given ``frame_id`` within simulation (at a specified ``distance`` in front of ``frame_id``).

.. list-table:: 
   :widths: 15 15 5 65
   :header-rows: 1
   :class: tight-table   
   
   * - Parameter
     - Type
     - Default
     - Description
   * - ``frame_id``
     - ``string``
     - ``base_link``
     - The frame Id to spawn the actor relative to.
   * - ``parent_frame_id``
     - ``string``
     - ``map``
     - The parent frame ID against which movement is evaluated.
   * - ``distance``
     - ``length``
     -
     - distance value relative to the frame_id at which to spawn the new actor
   * - ``world_name``
     - ``string``
     - ``default``
     - Gazebo world name
   * - ``model``
     - ``string``
     -
     - Model definition
   * - ``xacro_arguments``
     - ``string``
     -
     - (optional) Comma-separated list of argument key:=value pairs

``osc_object.spawn()``
""""""""""""""""""""""

Spawn an actor within simulation.

.. list-table:: 
   :widths: 15 15 5 65
   :header-rows: 1
   :class: tight-table   
   
   * - Parameter
     - Type
     - Default
     - Description
   * - ``spawn_pose: pose_3d``
     - ``pose_3d``
     -
     - Pose of the spawned actor.
   * - ``model``
     - ``string``
     - 
     - Model definition
   * - ``world_name``
     - ``string``
     - ``default``
     - Gazebo world name
   * - ``xacro_arguments``
     - ``string``
     -
     - (optional) Comma-separated list of argument key:=value pairs


.. note::

    The model definition can be specified in different formats:

    - ``file://<path-to-model>``: Local path to model file
    - ``model://<path-to-model>``: Path relative to available model search paths
    - ``<package-name>://<path-to-model>``: Path relative to an available package (e.g. :repo_link:`test/scenario_execution_gazebo_test/scenarios/test_spawn_exists_delete.osc`)
    - ``https:://fuel``: Model from `fuel.gazebosim.org <https://app.gazebosim.org/>`__ (e.g. ``https://fuel.gazebosim.org/1.0/OpenRobotics/models/Beer``)

    If the file ending is ``.xacro`` the model is forwarded to `xacro <https://wiki.ros.org/xacro>`__ before getting spawned.

``wait_for_sim()``
""""""""""""""""""
Wait for simulation to become active (checks for simulation clock).

.. list-table:: 
   :widths: 15 15 5 65
   :header-rows: 1
   :class: tight-table   
   
   * - Parameter
     - Type
     - Default
     - Description
   * - ``timeout``
     - ``time``
     - ``60s``
     - time to wait for the simulation. return failure afterwards.
   * - ``world_name``
     - ``string``
     - ``default``
     - Gazebo world name


Helpers
-------

The library contains basic helper methods. Import it with ``import osc.helpers``.

Actions
^^^^^^^

``log()``
"""""""""

For debugging purposes, log a string using the available log mechanism.

.. list-table:: 
   :widths: 15 15 5 65
   :header-rows: 1
   :class: tight-table   
   
   * - Parameter
     - Type
     - Default
     - Description
   * - ``msg``
     - ``string``
     -
     - String to log

``run_process()``
""""""""""""""""""""""""""

Run a process. Reports `running` while the process has not finished.

If ``wait_for_shutdown`` is ``false`` and the process is still running on scenario shutdown, ``shutdown_signal`` is sent. If the process does not shutdown within shutdown_timeout, ``signal.sigkill`` is sent.

.. list-table:: 
   :widths: 15 15 5 65
   :header-rows: 1
   :class: tight-table   
   
   * - Parameter
     - Type
     - Default
     - Description
   * - ``command``
     - ``string``
     -
     - Command to execute
   * - ``wait_for_shutdown``
     - ``bool``
     - ``true``
     - Wait for the process to be finished. If false, the action immediately finishes
   * - ``shutdown_signal``
     - ``signal``
     - ``signal!sigterm``
     - (Only used if ``wait_for_shutdown`` is ``false``) Signal that is sent if a process is still running on scenario shutdown
   * - ``shutdown_timeout``
     - ``time``
     - ``10s``
     - (Only used if ``wait_for_shutdown`` is ``false``) time to wait between ``shutdown_signal`` and SIGKILL getting sent, if process is still running on scenario shutdown

OS
--

The library contains actions to interact with the operating system. Import it with ``import osc.os``. It is provided by the package :repo_link:`libs/scenario_execution_os`.

Actions
^^^^^^^

``check_file_exists()``
"""""""""""""""""""""""

Report success if a file exists.

.. list-table:: 
   :widths: 15 15 5 65
   :header-rows: 1
   :class: tight-table   
   
   * - Parameter
     - Type
     - Default
     - Description
   * - ``file_name``
     - ``string``
     -
     - File to check


``check_file_not_exists()``
"""""""""""""""""""""""""""

Report success if a file does not exist.

.. list-table:: 
   :widths: 15 15 5 65
   :header-rows: 1
   :class: tight-table   
   
   * - Parameter
     - Type
     - Default
     - Description
   * - ``file_name``
     - ``string``
     -
     - File to check


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

.. list-table:: 
   :widths: 15 15 5 65
   :header-rows: 1
   :class: tight-table   
   
   * - Parameter
     - Type
     - Default
     - Description
   * - ``action_name``
     - ``string``
     -
     - Name of the action to connect to
   * - ``action_type``
     - ``string``
     -
     - Class of the action type (e.g. ``example_interfaces.action.Fibonacci``)
   * - ``data``
     - ``string``
     - 
     - Call content (e.g. ``{\"order\": 3}``)

``assert_lifecycle_state()``
""""""""""""""""""""""""""""
Checks for the state of a `lifecycle-managed <https://design.ros2.org/articles/node_lifecycle.html>`__ node.

.. list-table:: 
   :widths: 15 15 5 65
   :header-rows: 1
   :class: tight-table   
   
   * - Parameter
     - Type
     - Default
     - Description
   * - ``node_name``
     - ``string``
     - 
     - Name of ``lifecycle-managed`` node.
   * - ``state_sequence``
     - ``list of lifecycle_state``
     - 
     - List of states that a node is expected to transition through. The last entry is the state that a node is expected to remain in. Allowed ``['unconfigured', 'inactive', 'active', 'finalized]`` (e.g. ``[lifecycle_state!inactive, lifecycle_state!active]``)
   * - ``allow_initial_skip``
     - ``bool``
     - ``false``
     - If true, allows skipping of states at the beginning of ``state_sequence`` without reporting failure. 
   * - ``fail_on_unexpected``
     - ``bool``
     - ``true``
     - If true and an unexpected transition or final state occurs, the action fails. Otherwise it succeed.
   * - ``keep_running``
     - ``bool``
     - ``true``
     - If true, the action keeps running while the last state in the state_sequence remains


``assert_tf_moving()``
""""""""""""""""""""""

Checks that a tf ``frame_id`` keeps moving in respect to a ``parent_frame_id``. If there is no movement within ``timeout`` the action ends, depending on ``fail_on_finish``, either with success or failure. Speeds below ``threshold_translation`` and ``threshold_rotation`` are discarded. By default the action waits for the first transform to get available before starting the timeout timer. This can be changed by setting ``wait_for_first_transform`` to ``false``. If the tf topics are not available on ``/tf`` and ``/tf_static`` you can specify a namespace by setting ``tf_topic_namespace``.

.. list-table:: 
   :widths: 15 15 5 65
   :header-rows: 1
   :class: tight-table   
   
   * - Parameter
     - Type
     - Default
     - Description
   * - ``frame_id``
     - ``string``
     -
     - The frame Id to check for movement.
   * - ``parent_frame_id``
     - ``string``
     - ``map``
     - The parent frame ID against which movement is evaluated.
   * - ``timeout``
     - ``time``
     - 
     - Timeout without movement.
   * - ``threshold_translation``
     - ``speed``
     - ``0.01mps``
     - Translation speed below this threshold is skipped.
   * - ``threshold_rotation``
     - ``angular_rate``
     - ``0.01radps``
     - Rotational speed below this threshold is skipped.
   * - ``fail_on_finish``
     - ``bool``
     - ``true``
     - If true and there is no movement, the action fails. Otherwise it succeeds.
   * - ``wait_for_first_transform``
     - ``bool``
     - ``true``
     - If true, start measuring only after first message is received.
   * - ``tf_topic_namespace``
     - ``string``
     - ``''``
     - namespace of `tf` and `tf_static` topic.
   * - ``use_sim_time``
     - ``bool``
     - ``false``
     - In simulation, we need to look up the transform at a different time as the scenario execution node is not allowed to use the sim time

``assert_topic_latency()``
""""""""""""""""""""""""""

Check the latency of the specified topic (in system time). If the check with ``comparison_operator`` gets true, the action ends, depending on ``fail_on_finish``, either with success or failure.

.. list-table:: 
   :widths: 15 15 5 65
   :header-rows: 1
   :class: tight-table   
   
   * - Parameter
     - Type
     - Default
     - Description
   * - ``topic_name``
     - ``string``
     -
     - Topic name to wait for message
   * - ``latency``
     - ``time``
     -
     - The time to compare.
   * - ``comparison_operator``
     - ``comparison_operator``
     - ``comparison_operator!le``
     - operator to compare latency time.
   * - ``fail_on_finish``
     - ``bool``
     - ``true``
     - If false action success, if comparison is true.
   * - ``rolling_average_count``
     - ``int``
     - ``1``
     - check for the latency over the x elements.
   * - ``wait_for_first_message``
     - ``bool``
     - ``true``
     - if true, start measuring only after first message is received.
   * - ``topic_type``
     - ``string``
     - 
     - Class of message type, only required when 'wait_for_first_message' is set to false (e.g. ``std_msgs.msg.String``)


``check_data()``
""""""""""""""""
Compare received topic messages using the given ``comparison_operator``, against the specified value. Either the whole message gets compared or a member defined by ``member_name``.

.. list-table:: 
   :widths: 15 15 5 65
   :header-rows: 1
   :class: tight-table   
   
   * - Parameter
     - Type
     - Default
     - Description
   * - ``topic_name``
     - ``string``
     - 
     - Name of the topic to connect to
   * - ``topic_type``
     - ``string``
     - 
     - Class of the message type (e.g. ``std_msgs.msg.String``)
   * - ``qos_profile``
     - ``qos_preset_profiles``
     - ``qos_preset_profiles!system_default``
     - QoS Preset Profile for the subscriber
   * - ``member_name``
     - ``string``
     - ``''``
     - Name of the type member to check. If empty, the whole type is checked
   * - ``expected_value``
     - ``string``
     - 
     - Expected value
   * - ``comparison_operator``
     - ``comparison_operator``
     - ``comparison_operator!eq``
     - The comparison operator to apply
   * - ``fail_if_no_data``
     - ``bool``
     - ``false``
     - return failure if there is no data yet
   * - ``fail_if_bad_comparison``
     - ``bool``
     - ``true``
     - return failure if comparison failed
   * - ``wait_for_first_message``
     - ``bool``
     - ``true``
     - start checking with the first received message after action execution. If false, the check is executed on the last received message.

``differential_drive_robot.init_nav2()``
""""""""""""""""""""""""""""""""""""""""

Initialize nav2.

.. list-table:: 
   :widths: 15 15 5 65
   :header-rows: 1
   :class: tight-table   
   
   * - Parameter
     - Type
     - Default
     - Description
   * - ``initial_pose``
     - ``pose_3d``
     -
     - The initial pose to set during initialization
   * - ``base_frame_id``
     - ``string``
     - ``base_link``
     - Base Frame ID
   * - ``use_initial_pose``
     - ``bool``
     - ``true``
     - If false, no initial_pose is needed (useful when using slam instead of amcl for localization)
   * - ``namespace_override``
     - ``string``
     - 
     - If set, it's used as namespace (instead of the associated actor's namespace)
   * - ``wait_for_initial_pose``
     - ``bool``
     - ``false``
     - If true the initial pose needs to be set externally (e.g. manually through rviz)

``differential_drive_robot.nav_through_poses()``
""""""""""""""""""""""""""""""""""""""""""""""""

Use nav2 to navigate through poses.

.. list-table:: 
   :widths: 15 15 5 65
   :header-rows: 1
   :class: tight-table   
   
   * - Parameter
     - Type
     - Default
     - Description
   * - ``goal_poses``
     - ``list of pose_3d``
     -
     - Goal poses to navigate through
   * - ``namespace_override``
     - ``string``
     - ``''``
     - If set, it's used as namespace (instead of the associated actor's namespace)
   * - ``monitor_progress``
     - ``bool``
     - ``true``
     - If yes, the action returns after the goal is reached or on failure. If no, the action returns after request.

``differential_drive_robot.nav_to_pose()``
""""""""""""""""""""""""""""""""""""""""""
Use nav2 to navigate to goal pose.

.. list-table:: 
   :widths: 15 15 5 65
   :header-rows: 1
   :class: tight-table   
   
   * - Parameter
     - Type
     - Default
     - Description
   * - ``goal_pose``
     - ``pose_3d``
     - 
     - Goal pose to navigate to
   * - ``namespace_override``
     - ``string``
     - 
     - If set, it's used as namespace (instead of the associated actor's namespace)
   * - ``action_topic``
     - ``string``
     - ``navigate_to_pose``
     - Action name
   * - ``monitor_progress``
     - ``bool``
     - ``true``
     - If yes, the action returns after the goal is reached or on failure. If no, the action returns after request.

``differential_drive_robot.odometry_distance_traveled()``
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""

Wait until a defined distance was traveled, based on odometry.

.. list-table:: 
   :widths: 15 15 5 65
   :header-rows: 1
   :class: tight-table   
   
   * - Parameter
     - Type
     - Default
     - Description
   * - ``distance``
     - ``length``
     -
     - Traveled distance at which the action succeeds.
   * - ``namespace_override``
     - ``string``
     - 
     - if set, it's used as namespace (instead of the associated actor's namespace)

``differential_drive_robot.tf_close_to()``
""""""""""""""""""""""""""""""""""""""""""

Wait until a TF frame is close to a defined reference point.

.. list-table:: 
   :widths: 15 15 5 65
   :header-rows: 1
   :class: tight-table   
   
   * - Parameter
     - Type
     - Default
     - Description
   * - ``namespace_override``
     - ``string``
     - 
     - if set, it's used as namespace (instead of the associated actor's namespace)
   * - ``reference_point``
     - ``position_3d``
     -
     - Reference point to measure to distance to (z is not considered)
   * - ``threshold``
     - ``length``
     - 
     - Distance at which the action succeeds.
   * - ``sim``
     - ``bool``
     - ``false``
     - In simulation, we need to look up the transform map --> base_link at a different time as the scenario execution node is not allowed to use the sim time
   * - ``robot_frame_id``
     - ``string``
     - ``base_link``
     - Defines the TF frame id of the robot

``log_check()``
"""""""""""""""
Wait for specific output in ROS log (i.e. ``/rosout`` topic). If any of the entries within ``values`` the action succeeds.

.. list-table:: 
   :widths: 15 15 5 65
   :header-rows: 1
   :class: tight-table   
   
   * - Parameter
     - Type
     - Default
     - Description
   * - ``module_name``
     - ``string``
     - ``[]``
     - if specified, a matching message must also match the module name
   * - ``values``
     - ``list of string``
     - 
     - list of strings (in python syntax, e.g. "[\'foo\', \'bar\']")

``record_bag()``
""""""""""""""""

Record a ROS bag, stored in directory ``output_dir`` defined by command-line parameter (default: '.').

A common topic to record is ``/scenario_execution/snapshots`` which publishes changes within the behavior tree.

.. list-table:: 
   :widths: 15 15 5 65
   :header-rows: 1
   :class: tight-table   
   
   * - Parameter
     - Type
     - Default
     - Description
   * - ``topics``
     - ``list of string``
     - 
     - List of topics to capture
   * - ``timestamp_suffix``
     - ``bool``
     - ``true``
     - Add a timestamp suffix to output directory name
   * - ``hidden_topics``
     - ``bool``
     - ``false``
     - Whether to record hidden topics
   * - ``storage``
     - ``string``
     - ``''``
     - Storage type to use (empty string: use ROS bag record default)
   * - ``use_sim_time``
     - ``bool``
     - ``false``
     - Use simulation time for message timestamps by subscribing to the /clock topic

``ros_launch()``
""""""""""""""""

Execute a ROS launch file.

.. list-table:: 
   :widths: 15 15 5 65
   :header-rows: 1
   :class: tight-table   
   
   * - Parameter
     - Type
     - Default
     - Description
   * - ``package_name``
     - ``string``
     - 
     - Package that contains the launch file
   * - ``launch_file``
     - ``string``
     - 
     - Launch file name
   * - ``arguments``
     - ``list of key_value``
     -
     - ROS arguments (get forwarded as key:=value pairs)
   * - ``wait_for_shutdown``
     - ``bool``
     - ``true``
     - If true, the action waits until the execution is finished
   * - ``shutdown_timeout``
     - ``time``
     - ``10s``
     - (Only used ``if wait_for_shutdown`` is ``false``) Time to wait between ``SIGINT`` and ``SIGKILL`` getting sent, if process is still running on scenario shutdown

``service_call()``
""""""""""""""""""

Call a ROS service and wait for the reply.

.. list-table:: 
   :widths: 15 15 5 65
   :header-rows: 1
   :class: tight-table   
   
   * - Parameter
     - Type
     - Default
     - Description
   * - ``service_name``
     - ``string``
     - 
     - Name of the service to connect to
   * - ``service_type``
     - ``string``
     - 
     - Class of the message type (e.g. ``std_srvs.msg.Empty``)
   * - ``data``
     - ``string``
     - 
     - Service call content

``set_node_parameter()``
""""""""""""""""""""""""

Set a parameter of a node.

.. list-table:: 
   :widths: 15 15 5 65
   :header-rows: 1
   :class: tight-table   
   
   * - Parameter
     - Type
     - Default
     - Description
   * - ``node_name``
     - ``string``
     - 
     - Name of the node
   * - ``parameter_name``
     - ``string``
     - 
     - Name of the parameter
   * - ``parameter_value``
     - ``string``
     - 
     - Value of the parameter

``topic_monitor()``
"""""""""""""""""""

Subscribe to a topic and store the last message within a variable.

.. list-table:: 
   :widths: 15 15 5 65
   :header-rows: 1
   :class: tight-table   
   
   * - Parameter
     - Type
     - Default
     - Description
   * - ``topic_name``
     - ``string``
     - 
     - name of the topic to monitor
   * - ``topic_type``
     - ``string``
     - 
     - class of the message type (e.g. ``std_msgs.msg.String``)
   * - ``target_variable``
     - ``variable``
     - 
     - variable to store the received value (e.g. a ``var`` within an actor instance)
   * - ``qos_profile``
     - ``qos_preset_profiles``
     - ``qos_preset_profiles!system_default``
     - QoS profile for the subscriber (default: ``qos_preset_profiles!system_default``)

``topic_publish()``
"""""""""""""""""""

Publish a message on a topic.

.. list-table:: 
   :widths: 15 15 5 65
   :header-rows: 1
   :class: tight-table   
   
   * - Parameter
     - Type
     - Default
     - Description
   * - ``topic_name``
     - ``string``
     - 
     - Name of the topic to publish to
   * - ``topic_type``
     - ``string``
     - 
     - Class of the message type (e.g. ``std_msgs.msg.String``)
   * - ``value``
     - ``string``
     - 
     - Value to publish (can either be a string that gets parsed, a struct or a message object stored within a variable)
   * - ``qos_profile``
     - ``qos_preset_profiles``
     - ``qos_preset_profiles!system_default``
     - QoS Preset Profile for the subscriber (default: ``qos_preset_profiles!system_default``)

``wait_for_data()``
"""""""""""""""""""

Wait for any message on a ROS topic.

.. list-table:: 
   :widths: 15 15 5 65
   :header-rows: 1
   :class: tight-table   
   
   * - Parameter
     - Type
     - Default
     - Description
   * - ``topic_name``
     - ``string``
     - 
     - Name of the topic to connect to
   * - ``topic_type``
     - ``string``
     - 
     - Class of the message type (e.g. ``std_msgs.msg.String``)
   * - ``qos_profile``
     - ``qos_preset_profiles``
     - ``qos_preset_profiles!system_default``
     - QoS Preset Profile for the subscriber (default: ``qos_preset_profiles!system_default``)


``wait_for_topics()``
"""""""""""""""""""""

Wait for topics to get available (i.e. publisher gets available).

.. list-table:: 
   :widths: 15 15 5 65
   :header-rows: 1
   :class: tight-table   
   
   * - Parameter
     - Type
     - Default
     - Description
   * - ``topics``
     - ``list of string``
     - 
     - List of topics to wait for
