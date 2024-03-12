Libraries
=========

Beside ``osc.standard`` provided by OpenSCENARIO 2, multiple libraries are provided with scenario execution.

- ``osc.helpers`` Helpers Library
- ``osc.robotics``: Robotics Library
- ``osc.ros``: ROS Library
- ``osc.gazebo``: Gazebo Library

Additional features can be implemented by defining your own library.

``osc.helpers``
---------------

Actions
^^^^^^^

``log()``
"""""""""

For debugging purposes, log a string using the available log mechanism.

- ``msg: string``: String to log

``run_external_process()``
""""""""""""""""""""""""""

Run an external process. Reports `running` while the process has not finished.

- ``command: string``: Command to execute


``osc.robotics``
----------------

Actors
^^^^^^

``differential_drive_robot``
""""""""""""""""""""""""""""
A differential drive robot actor.

``osc.ros``
-----------

Actions
^^^^^^^

``wait_for_data()``
"""""""""""""""""""

Wait for a specific data on a ROS topic.

- ``topic_name: string``: Name of the topic to connect to
- ``topic_type: string``: Class of the message type (e.g. ``std_msgs.msg.String``)
- ``qos_profile: qos_preset_profiles``: QoS Preset Profile for the subscriber (default: ``qos_preset_profiles!system_default``)
- ``clearing_policy: clearing_policy``: When to clear the data (default: ``clearing_policy!on_initialise``)


``wait_for_topics()``
"""""""""""""""""""""

Wait for topics to get available (i.e. publisher gets available).

- ``topics: list of string``: List of topics to wait for


``check_data()``
""""""""""""""""
Wait for a topic message, compare a message field against a specific value

- ``topic_name: string``: Name of the topic to connect to
- ``topic_type: string``: Class of the message type (e.g. ``std_msgs.msg.String``)
- ``qos_profile: qos_preset_profiles``: QoS Preset Profile for the subscriber (default: ``qos_preset_profiles!system_default``)
- ``variable_name: string``: Name of the variable to check
- ``expected_value: string``: Expected value of the variable
- ``comparison_operator: comparison_operator``: The comparison operator to apply (default: ``comparison_operator!eq``)
- ``fail_if_no_data: bool``: py_trees.common.Status.FAILURE instead of py_trees.common.Status.RUNNING if there is no data yet (default: ``false``)
- ``fail_if_bad_comparison: bool``: py_trees.common.Status.FAILURE instead of py_trees.common.Status.RUNNING if comparison failed (default: ``true``)
- ``clearing_policy: clearing_policy``: When to clear the data (default: ``clearing_policy!on_initialise``)

``service_call()``
""""""""""""""""""

Call a ROS service and wait for the reply.

- ``service_name: string``: Name of the service to connect to
- ``service_type: string``: Class of the message type (e.g. ``std_srvs.msg.Empty``)
- ``data: string``: Service call content

``topic_publish()``
"""""""""""""""""""

Publish a message on a topic.

- ``topic_name: string``: Name of the topic to publish to
- ``topic_type: string``: Class of the message type (e.g. ``std_msgs.msg.String``)
- ``qos_profile: qos_preset_profiles``: QoS Preset Profile for the subscriber (default: ``qos_preset_profiles!system_default``)
- ``value: string``: Value to publish

``set_node_parameter()``
""""""""""""""""""""""""

Set a parameter of a node.

- ``node_name: string``: Name of the node
- ``parameter_name: string``: Name of the parameter
- ``parameter_value: string``: Value of the parameter

``record_bag()``
""""""""""""""""

Record a ROS bag, stored in directory ``output_dir`` defined by command-line parameter (default: '.')

- ``topics: list of string``: List of topics to capture
- ``timestamp_suffix: bool``: Add a timestamp suffix to output directory name (default: ``true``)
- ``hidden_topics: bool``: Whether to record hidden topics (default: ``false``)
- ``storage: string``: Storage type to use (empty string: use ROS bag record default)

``log_check()``
"""""""""""""""
Wait for specific output in ROS log (i.e. `/rosout` topic). If any of the entries within ``values`` the action succeeds.

- ``module_name: string``: if specified, a matching message must also match the module name (default: empty)
- ``values: list of string``: list of strings (in python syntax, e.g. "[\'foo\', \'bar\']")

``differential_drive_robot.init_nav2()``
""""""""""""""""""""""""""""""""""""""""

Initialize nav2.

- ``initial_pose: pose_3d``: The initial pose to set during initialization
- ``base_frame_id: string``: Base Frame ID (default: ``base_link``)
- ``use_initial_pose: bool``: If false, no initial_pose is needed (useful when using slam instead of amcl for localization) (default: ``true``)
- ``namespace_override: string``: If set, it's used as namespace (instead of the associated actor's namespace)
- ``wait_for_initial_pose: bool``: If true the initial pose needs to be set externally (e.g. manually through rviz)(default: ``false``)

``differential_drive_robot.nav_to_pose()``
""""""""""""""""""""""""""""""""""""""""""
Use nav2 to navigate to goal pose.

- ``goal_pose: pose_3d``: Goal pose to navigate to
- ``namespace_override: string``: If set, it's used as namespace (instead of the associated actor's namespace)
- ``action_topic: string``: Action name (default: ``navigate_to_pose``)
- ``monitor_progress: bool``: If yes, the action returns after the goal is reached or on failure. If no, the action returns after request. (default: ``true``)

``differential_drive_robot.nav_through_poses()``
""""""""""""""""""""""""""""""""""""""""""""""""

Use nav2 to navigate through poses.

- ``goal_pose: string``: Goal poses to navigate to (format: ``x1,y1,yaw1;x2,y2,yaw2;...``)
- ``namespace_override: string``: If set, it's used as namespace (instead of the associated actor's namespace)
- ``monitor_progress: bool``: If yes, the action returns after the goal is reached or on failure. If no, the action returns after request. (default: ``true``)

``differential_drive_robot.tf_close_to()``
""""""""""""""""""""""""""""""""""""""""""

Wait until a TF frame is close to a defined reference point.

- ``namespace_override: string``: if set, it's used as namespace (instead of the associated actor's namespace)
- ``reference_point: position_3d``: Reference point to measure to distance to (z is not considered)
- ``threshold: length``: Distance at which the action succeeds.
- ``sim: bool``: In simulation, we need to look up the transform map --> base_link at a different time as the scenario execution node is not allowed to use the sim time (default: ``false``)
- ``robot_frame_id: string``: Defines the TF frame id of the robot (default: ``base_link``)

``differential_drive_robot.odometry_distance_traveled()``
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""

Wait until a defined distance was traveled, based on odometry.

- ``namespace: string``:  Namespace of the odometry topic
- ``distance: length``: Traveled distance at which the action succeeds.


``osc.gazebo``
--------------

Actions
^^^^^^^


``wait_for_sim()``
""""""""""""""""""
Wait for simulation to become active (checks for simulation clock).

- ``world_name: string``: Gazebo world name (default: ``default``)

``actor_exists()``
""""""""""""""""""

Waits for an actor to exist within simulation.

- ``entity_name``: Entity name within simulation
- ``world_name: string``: Gazebo world name (default: ``default``)

``osc_object.spawn()``
""""""""""""""""""""""

Spawn an actor within simulation by using the ``model`` and ``namespace`` of the associated actor.

- ``spawn_pose: pose_3d``: Pose of the spawned actor.
- ``world_name: string``: Gazebo world name (default: ``default``)
- ``xacro_arguments: string``: Comma-separated list of argument key:=value pairs

``osc_object.delete()``
"""""""""""""""""""""""

Delete an object from the simulation.

- ``world_name: string``: Gazebo world name (default: ``default``)
