Libraries
=========

Beside ``osc.standard`` and ``osc.types`` provided by OpenSCENARIO DSL, multiple libraries are provided with scenario execution.

.. list-table:: 
   :widths: 40 60
   :header-rows: 1
   :class: tight-table   
   
   * - Name
     - Description
   * - ``osc.docker``
     - Docker Library (provided with :repo_link:`libs/scenario_execution_docker`)
   * - ``osc.gazebo``
     - Gazebo Library (provided with :repo_link:`libs/scenario_execution_gazebo`)
   * - ``osc.helpers``
     - Helpers Library (provided with :repo_link:`scenario_execution`)
   * - ``osc.kubernetes``
     - Kubernetes Library (provided with :repo_link:`libs/scenario_execution_kubernetes`)
   * - ``osc.moveit2``
     - ROS Moveit2  manipulation stack Library (provided with :repo_link:`libs/scenario_execution_moveit2`)
   * - ``osc.nav2``
     - ROS Nav2 navigation stack Library (provided with :repo_link:`libs/scenario_execution_nav2`)
   * - ``osc.os``
     - Library to interact with the operating system (provided with :repo_link:`libs/scenario_execution_os`)
   * - ``osc.robotics``
     - Robotics Library (provided with :repo_link:`scenario_execution`)
   * - ``osc.ros``
     - ROS Library (provided with :repo_link:`scenario_execution_ros`)
   * - ``osc.x11``
     - X11 Library (provided with :repo_link:`libs/scenario_execution_x11`)

Additional features can be implemented by defining your own library.


Docker
------

The library contains actions to interact with `Docker <https://www.docker.com/>`_. Import it with ``import osc.docker``. It's provided by the package :repo_link:`libs/scenario_execution_docker`.

``docker_run()``
^^^^^^^^^^^^^^^^

Runs a Docker container

.. list-table:: 
   :widths: 15 15 5 65
   :header-rows: 1
   :class: tight-table   
   
   * - Parameter
     - Type
     - Default
     - Description
   * - ``image``
     - ``string``
     -
     - The image to run
   * - ``command``
     - ``string``
     - 
     - The command to run in the container
   * - ``container_name``
     - ``string``
     - 
     - The name for this container
   * - ``detach``
     - ``bool``
     - false
     - Whether to run container in the background
   * - ``environment``
     - ``list of string``
     - 
     - Environment variables to set inside the container, i.e., a list of strings in the format ["SOMEVARIABLE=xxx"].
   * - ``network``
     - ``string``
     - 
     - Name of the network this container will be connected to at creation time
   * - ``privileged``
     - ``bool``
     - false
     - Give extended privileges to this container
   * - ``remove``
     - ``bool``
     - true
     - Remove the container when it as finished running
   * - ``stream``
     - ``bool``
     - true
     - If true and detach is false, return a log generator instead of a string. Ignored if detach is true.
   * - ``volumes``
     - ``list of string``
     - 
     - A list of strings which each one of its elements specifies a mount volume: ['/home/user1/:/mount/vol2','/home/user2/:/mount/vol1']

``docker_exec()``
^^^^^^^^^^^^^^^^^

Runs a command inside a given Docker container

.. list-table:: 
   :widths: 15 15 5 65
   :header-rows: 1
   :class: tight-table   
   
   * - Parameter
     - Type
     - Default
     - Description
   * - ``container``
     - ``string``
     - 
     - The name or id of the container to run the command in 
   * - ``container``
     - ``string``
     - 
     - The name or id of the container to run the command in 
   * - ``command``
     - ``string``
     - 
     - The command to run inside the container
   * - ``environment``
     - ``list of string``
     - 
     - Environment variables to set inside the container, i.e., a list of strings in the format ["SOMEVARIABLE=xxx"].
   * - ``privileged``
     - ``bool``
     - false
     - Give extended privileges to this container
   * - ``user``
     - ``string``
     - root
     - User to execute command as
   * - ``workdir``
     - ``string``
     - 
     - Path to working directory for this exec session

``docker_copy()``
^^^^^^^^^^^^^^^^^

Copy a file or folder from the container.
Note that this actions potentially blocks other action calls if the copied content is large.
In case large files or folders need to be copied, consider mounting a volume to the container instead of this action.

.. list-table:: 
   :widths: 15 15 5 65
   :header-rows: 1
   :class: tight-table   
   
   * - Parameter
     - Type
     - Default
     - Description
   * - ``container``
     - ``string``
     - 
     - The name or id of the container to run the command in 
   * - ``file_path``
     - ``string``
     - 
     - Path to the file or folder inside the container to retrieve

``docker_put()``
^^^^^^^^^^^^^^^^^

Copy a file or folder from the local system into a running container.
Note that this actions potentially blocks other action calls if the copied content is large.
In case large files or folders need to be copied, consider mounting a volume to the container instead of this action.

.. list-table:: 
   :widths: 15 15 5 65
   :header-rows: 1
   :class: tight-table   
   
   * - Parameter
     - Type
     - Default
     - Description
   * - ``container``
     - ``string``
     - 
     - The name or id of the container to put the file or folder into 
   * - ``source_path``
     - ``string``
     - 
     - Path to the file or folder in the local system to copy
   * - ``target_path``
     - ``string``
     - 
     - Target path inside the container to put the file or folder
    
Gazebo
------

The library contains actions to interact with the `Gazebo Simulation <https://gazebosim.org/>`_. Import it with ``import osc.gazebo``. It's provided by the package :repo_link:`libs/scenario_execution_gazebo`.

``actor_exists()``
^^^^^^^^^^^^^^^^^^

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
^^^^^^^^^^^^^^^^^^^^^^^

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
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
^^^^^^^^^^^^^^^^^^^^^^

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
^^^^^^^^^^^^^^^^^^

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

Modifiers
^^^^^^^^^

``inverter()``
""""""""""""""

Modifier to invert the action result. A failing action will report ``success``, a succeeding action will report ``failure``.

``repeat()``
""""""""""""
Modifier to repeat a sub-tree. If any of the included children report ``failure``, the repetition stops and ``failure`` is reported.

.. list-table:: 
   :widths: 15 15 5 65
   :header-rows: 1
   :class: tight-table   

   * - Parameter
     - Type
     - Default
     - Description
   * - ``count``
     - ``int``
     - ``-1``
     - Repeat this many times (-1 to repeat indefinitely)

``retry()``
"""""""""""
Modifier to retry a sub-tree until it succeeds.

.. list-table:: 
   :widths: 15 15 5 65
   :header-rows: 1
   :class: tight-table   

   * - Parameter
     - Type
     - Default
     - Description
   * - ``count``
     - ``int``
     - 
     - Maximum number of permitted failures

``timeout()``
"""""""""""""
Modifier to set a timeout for a sub-tree.

.. list-table:: 
   :widths: 15 15 5 65
   :header-rows: 1
   :class: tight-table   

   * - Parameter
     - Type
     - Default
     - Description
   * - ``duration``
     - ``time``
     - 
     - Time to wait

``failure_is_running()``
""""""""""""""""""""""""

Don't stop running.

``failure_is_success()``
""""""""""""""""""""""""

Be positive, always succeed.

``running_is_failure()``
""""""""""""""""""""""""

Got to be snappy! We want results...yesterday.

``running_is_success()``
""""""""""""""""""""""""

Don't hang around...

``success_is_failure()``
""""""""""""""""""""""""

Be depressed, always fail.

``success_is_running()``
""""""""""""""""""""""""

The tickling never ends...


``compare()``
^^^^^^^^^^^^^

Compare two values. If the comparison is true, the action is successful.

.. list-table:: 
   :widths: 15 15 5 65
   :header-rows: 1
   :class: tight-table   
   
   * - Parameter
     - Type
     - Default
     - Description
   * - ``left_value``
     - ``string``
     -
     - Left value of comparison
   * - ``operator``
     - ``string``
     -
     - Possible operator string values: ``==``, ``!=``, ``<``, ``<=``, ``>``, ``>=``
   * - ``right_value``
     - ``string``
     -
     - Right value of comparison



``decrement()``
^^^^^^^^^^^^^^^

Decrement the value of a variable.

.. list-table:: 
   :widths: 15 15 5 65
   :header-rows: 1
   :class: tight-table   
   
   * - Parameter
     - Type
     - Default
     - Description
   * - ``target_variable``
     - ``variable``
     -
     - Variable to decrement


``increment()``
^^^^^^^^^^^^^^^

Increment the value of a variable.

.. list-table:: 
   :widths: 15 15 5 65
   :header-rows: 1
   :class: tight-table   
   
   * - Parameter
     - Type
     - Default
     - Description
   * - ``target_variable``
     - ``variable``
     -
     - Variable to increment


``log()``
^^^^^^^^^

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
^^^^^^^^^^^^^^^^^

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


Kubernetes
----------

The library contains actions to interact with the `Kubernetes API <https://kubernetes.io>`_. Import it with ``import osc.kubernetes``. It's provided by the package :repo_link:`libs/scenario_execution_kubernetes`.

``kubernetes_create_from_yaml()``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Create a Kubernetes object (e.g., a pod or network policy) from a yaml file. 

.. list-table:: 
   :widths: 15 15 5 65
   :header-rows: 1
   :class: tight-table   
   
   * - Parameter
     - Type
     - Default
     - Description
   * - ``namespace``
     - ``string``
     - ``default``
     - Kubernetes namespace
   * - ``within_cluster``
     - ``bool``
     - ``false``
     - set to true if you want to access the cluster from within a running container/pod
   * - ``yaml_file``
     - ``string``
     - 
     - The yaml-file to use create the object from


``kubernetes_delete()``
^^^^^^^^^^^^^^^^^^^^^^^

Delete a Kubernetes element (e.g., a pod or network policy).

.. list-table:: 
   :widths: 15 15 5 65
   :header-rows: 1
   :class: tight-table   
   
   * - Parameter
     - Type
     - Default
     - Description
   * - ``namespace``
     - ``string``
     - ``default``
     - Kubernetes namespace
   * - ``within_cluster``
     - ``bool``
     - ``false``
     - set to true if you want to access the cluster from within a running container/pod
   * - ``target``
     - ``string``
     - 
     - The target element to delete
   * - ``regex``
     - ``bool``
     - ``false``
     - Is the specified target a regular expression
   * - ``element_type``
     - ``kubernetes_element_type``
     - 
     - Type of the element to delete (e.g., ``kubernetes_element_type!pod``)
   * - ``grace_period``
     - ``time``
     - ``5s``
     - Grace period to wait before forcing deletion


``kubernetes_patch_network_policy()``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Patch an existing Kubernetes network policy.

.. list-table:: 
   :widths: 15 15 5 65
   :header-rows: 1
   :class: tight-table   
   
   * - Parameter
     - Type
     - Default
     - Description
   * - ``namespace``
     - ``string``
     - ``default``
     - Kubernetes namespace
   * - ``within_cluster``
     - ``bool``
     - ``false``
     - set to true if you want to access the cluster from within a running container/pod
   * - ``target``
     - ``string``
     - 
     - The target network policy to patch
   * - ``ingress_enabled``
     - ``bool``
     - 
     - Should ingress (i.e., incoming) network traffic be enabled
   * - ``egress_enabled``
     - ``bool``
     - 
     - Should egress (i.e., outgoing) network traffic be enabled
   * - ``match_label``
     - ``key_value``
     - 
     - key-value pair to match (e.g., ``key_value("app", "pod_name"))``


``kubernetes_patch_pod()``
^^^^^^^^^^^^^^^^^^^^^^^^^^

Patch an existing pod. If patching resources, please check `feature gates <https://kubernetes.io/docs/tasks/configure-pod-container/resize-container-resources/#container-resize-policies>`__

.. list-table:: 
   :widths: 15 15 5 65
   :header-rows: 1
   :class: tight-table   
   
   * - Parameter
     - Type
     - Default
     - Description
   * - ``namespace``
     - ``string``
     - ``default``
     - Kubernetes namespace
   * - ``within_cluster``
     - ``bool``
     - ``false``
     - set to true if you want to access the cluster from within a running container/pod
   * - ``target``
     - ``string``
     - 
     - The target pod to patch
   * - ``body``
     - ``string``
     - 
     - Patch to apply. Example: ``'{\"spec\":{\"containers\":[{\"name\":\"main\", \"resources\":{\"requests\":{\"cpu\":\"200m\"}, \"limits\":{\"cpu\":\"200m\"}}}]}}'``


``kubernetes_pod_exec()``
^^^^^^^^^^^^^^^^^^^^^^^^^

Execute a command within a running pod

.. list-table:: 
   :widths: 15 15 5 65
   :header-rows: 1
   :class: tight-table   
   
   * - Parameter
     - Type
     - Default
     - Description
   * - ``namespace``
     - ``string``
     - ``default``
     - Kubernetes namespace
   * - ``within_cluster``
     - ``bool``
     - ``false``
     - set to true if you want to access the cluster from within a running container/pod
   * - ``target``
     - ``string``
     - 
     - The target pod to execute the command in 
   * - ``command``
     - ``list of string``
     - 
     - Command to execute
   * - ``regex``
     - ``bool``
     - ``false``
     - Is the specified target a regular expression


``kubernetes_wait_for_network_policy_status()``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Wait for an existing Kubernetes network policy to reach a specified state.

.. list-table:: 
   :widths: 15 15 5 65
   :header-rows: 1
   :class: tight-table   
   
   * - Parameter
     - Type
     - Default
     - Description
   * - ``namespace``
     - ``string``
     - ``default``
     - Kubernetes namespace
   * - ``within_cluster``
     - ``bool``
     - ``false``
     - set to true if you want to access the cluster from within a running container/pod
   * - ``target``
     - ``string``
     - 
     - The target network policy to monitor
   * - ``status``
     - ``kubernetes_network_policy_status``
     - 
     - Expected status of the network policy, e.g., ``kubernetes_network_policy_status!added``


``kubernetes_wait_for_pod_status()``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Wait for a Kubernetes pod to reach a specified state.

.. list-table:: 
   :widths: 15 15 5 65
   :header-rows: 1
   :class: tight-table   
   
   * - Parameter
     - Type
     - Default
     - Description
   * - ``namespace``
     - ``string``
     - ``default``
     - Kubernetes namespace
   * - ``within_cluster``
     - ``bool``
     - ``false``
     - set to true if you want to access the cluster from within a running container/pod
   * - ``target``
     - ``string``
     - 
     - The name of the pod to monitor
   * - ``status``
     - ``kubernetes_pod_status``
     - 
     - Expected status of the pod, e.g., ``kubernetes_pod_status!running``
   * - ``regex``
     - ``bool``
     - ``false``
     - Is the specified target a regular expression


Moveit2
-------

The library contains actions to interact with the `Moveit2 <https://moveit.picknik.ai/main/index.html>`__ manipulation stack. Import it with ``import osc.moveit2``. It is provided by the package :repo_link:`libs/scenario_execution_moveit2`.

Actors
^^^^^^

``arm``
"""""""
An articulated arm actor inheriting from the more general ``robot`` actor

.. list-table:: 
   :widths: 15 15 5 65
   :header-rows: 1
   :class: tight-table   
   
   * - Parameter
     - Type
     - Default
     - Description
   * - ``namespace``
     - ``string``
     - `` ' ' ``
     - Namespace for the arm
   * - ``arm_joints``
     - ``list of string``
     -
     - List of joint names for the arm joints
   * - ``gripper_joints``
     - ``list of string``
     -
     - List of joint names for the gripper joints
   * - ``arm_group``
     - ``bool``
     - ``false``
     - Name of the move group controlling the arm joints
   * - ``gripper_group``
     - ``string``
     - 
     - Name of the move group controlling the gripper joints
   * - ``end_effector``
     - ``string``
     -
     - Name of the end effector component (e.g., hand or tool)
   * - ``base_link``
     - ``string``
     -
     - Name of the robot's base link for reference in kinematics

``arm.move_to_joint_pose()``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Use MoveIt2 to move the arm joints to specified joint positions, utilizing `MoveGroup action <https://github.com/moveit/moveit_msgs/blob/master/action/MoveGroup.action>`__ from the move_group node by specifying target joint values.

.. list-table:: 
   :widths: 15 15 5 65
   :header-rows: 1
   :class: tight-table   
   
   * - Parameter
     - Type
     - Default
     - Description
   * - ``goal_pose``
     - ``list of float``
     -
     - List joint positions to move to
   * - ``move_group``
     - ``move_group_type``
     -
     - Move group type. Allowed [arm, gripper] (e.g. ``[move_group_type!arm, move_group_type!gripper]``)
   * - ``plan_only``
     - ``bool``
     - ``false``
     - If true, the plan is calculated but not executed. The calculated plan can be visualized in rviz.
   * - ``replan``
     - ``bool``
     - ``true``
     - If true, replan if plan becomes invalidated during execution
   * - ``tolerance``
     - ``float``
     - ``0.001``
     - The acceptable range of variation around both the start and goal positions.
   * - ``max_velocity_scaling_factor``
     - ``float``
     - ``0.1``
     - Scaling factors for optionally reducing the maximum joint velocities
   * - ``namespace_override``
     - ``string``
     - ``false``
     - if set, it's used as namespace (instead of the associated actor's name)
   * - ``action_topic``
     - ``string``
     - ``move_action``
     - Action name
   * - ``success_on_acceptance``
     - ``bool``
     - ``false``
     - Succeed on goal acceptance

``arm.move_to_pose``
^^^^^^^^^^^^^^^^^^^^

Use MoveIt2 to move the end-effector to a specified pose, utilizing `MoveGroup action <https://github.com/moveit/moveit_msgs/blob/master/action/MoveGroup.action>`__ from the move_group node by specifying the desired end-effector position and orientation.

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
     - end effector pose to move to
   * - ``plan_only``
     - ``bool``
     - ``false``
     - If true, the plan is calculated but not executed. The calculated plan can be visualized in rviz.
   * - ``replan``
     - ``bool``
     - ``true``
     - If true, replan if plan becomes invalidated during execution
   * - ``tolerance``
     - ``float``
     - ``0.001``
     - The acceptable range of variation around both the start and goal positions.
   * - ``max_velocity_scaling_factor``
     - ``float``
     - ``0.1``
     - Scaling factors for optionally reducing the maximum joint velocities
   * - ``namespace_override``
     - ``string``
     - ``false``
     - if set, it's used as namespace (instead of the associated actor's name)
   * - ``action_topic``
     - ``string``
     - ``move_action``
     - Action name
   * - ``success_on_acceptance``
     - ``bool``
     - ``false``
     - Succeed on goal acceptance


Nav2
----

The library contains actions to interact with the `Nav2 <https://docs.nav2.org/>`__ navigation stack. Import it with ``import osc.nav2``. It is provided by the package :repo_link:`libs/scenario_execution_nav2`.

``differential_drive_robot.init_nav2()``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
   * - ``wait_for_amcl``
     - ``bool``
     - ``true``
     - If true, wait for amcl localization to be ready

``differential_drive_robot.nav_through_poses()``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
   * - ``action_topic``
     - ``string``
     - ``navigate_through_poses``
     - Action name
   * - ``success_on_acceptance``
     - ``bool``
     - ``false``
     -  succeed on goal acceptance

``differential_drive_robot.nav_to_pose()``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
   * - ``success_on_acceptance``
     - ``bool``
     - ``false``
     -  succeed on goal acceptance

OS
--

The library contains actions to interact with the operating system. Import it with ``import osc.os``. It is provided by the package :repo_link:`libs/scenario_execution_os`.

External Methods
^^^^^^^^^^^^^^^^

.. list-table:: 
   :widths: 30 70
   :header-rows: 1
   :class: tight-table   
   
   * - External Method
     - Description
   * - ``abspath(path: string)``
     - Return a normalized absolutized version of the path-name ``path``.
   * - ``basename(p: string)``
     - Return the base name of path-name ``p``.
   * - ``dirname(p: string)``
     - Return the directory name of path-name ``p``.


``check_file_exists()``
^^^^^^^^^^^^^^^^^^^^^^^

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


``action_call()``
^^^^^^^^^^^^^^^^^

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
   * - ``success_on_acceptance``
     - ``bool``
     - ``false``
     -  succeed on goal acceptance

``assert_lifecycle_state()``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
^^^^^^^^^^^^^^^^^^^^^^

Checks that a tf ``frame_id`` keeps moving in respect to a ``parent_frame_id``. If there is no movement within ``timeout`` the action with failure. Speeds below ``threshold_translation`` and ``threshold_rotation`` are discarded. By default the action waits for the first transform to get available before starting the timeout timer. This can be changed by setting ``wait_for_first_transform`` to ``false``. If the tf topics are not available on ``/tf`` and ``/tf_static`` you can specify a namespace by setting ``tf_topic_namespace``.

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
^^^^^^^^^^^^^^^^^^^^^^^^^^

Check the latency of the specified topic (in system time). If the check with ``comparison_operator`` gets true, the action ends with failure.

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


``bag_play()``
^^^^^^^^^^^^^^^

Play back a ROS bag.

.. list-table:: 
   :widths: 15 15 5 65
   :header-rows: 1
   :class: tight-table   
   
   * - Parameter
     - Type
     - Default
     - Description
   * - ``source``
     - ``string``
     - 
     - path to ROS bag directory, either absolute or relative to scenario-file directory
   * - ``topics``
     - ``list of string``
     - 
     - topics to publish, if empty all topics are published
   * - ``publish_clock``
     - ``bool``
     - ``false``
     - whether to publish to /clock
   * - ``publish_clock_rate``
     - ``float``
     - ``1.0``
     - if ``publish_clock`` is true, publish to ``/clock`` at the specified frequency in Hz, to act as a ROS Time Source.
   * - ``start_offset``
     - ``float``
     - ``0.0``
     - start the playback this many seconds into the bag file


``bag_record()``
^^^^^^^^^^^^^^^^

Record a ROS bag, stored in directory ``output_dir`` defined by command-line parameter (default: ``.``). If ``topics`` is specified, this action waits for all topics to be subscribed until it returns with success otherwise it immediately returns. The recording is active until the end of the scenario.

A common topic to record is ``/scenario_execution/snapshots`` which publishes changes within the behavior tree. When replaying the bag-file, this allows to visualize the current state of the scenario in RViz, using the ``scenario_execution_rviz`` plugin.

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


``check_data()``
^^^^^^^^^^^^^^^^

Compare received topic messages using the given ``comparison_operator``, against the specified value. Either the whole message gets compared or a member defined by ``member_name``. If the ``expected_value`` is a string, set ``eval_expected_value`` to ``true``.

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
   * - ``expected_value``
     - ``string``
     - 
     - Expected value
   * - ``eval_expected_value``
     - ``bool``
     - ``true``
     - Should the expected value get evaluated (using ``ast.literal_eval()``). Set to ``false`` if expected value is a string
   * - ``qos_profile``
     - ``qos_preset_profiles``
     - ``qos_preset_profiles!system_default``
     - QoS Preset Profile for the subscriber
   * - ``member_name``
     - ``string``
     - ``''``
     - Name of the type member to check. If empty, the whole type is checked
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


``check_data_external()``
^^^^^^^^^^^^^^^^^^^^^^^^^

Compare received topic messages using an external python function ``function_name`` defined in python file ``file_path`` relative to the scenario-file.

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
   * - ``file_path``
     - ``string``
     - 
     - Path to python file containing the external check function
   * - ``function_name``
     - ``string``
     - 
     - python function to be called. The function is expected to have the signature: ``def function_name(msg) -> bool``
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


``differential_drive_robot.odometry_distance_traveled()``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Wait until a TF frame is close to a defined reference point.

.. list-table:: 
   :widths: 15 15 5 65
   :header-rows: 1
   :class: tight-table   
   
   * - Parameter
     - Type
     - Default
     - Description
   * - ``threshold``
     - ``length``
     - 
     - Distance at which the action succeeds.
   * - ``reference_point``
     - ``position_3d``
     -
     - Reference point to measure to distance to (z is not considered)
   * - ``robot_frame_id``
     - ``string``
     - ``base_link``
     - Defines the TF frame id of the robot
   * - ``sim``
     - ``bool``
     - ``false``
     - In simulation, we need to look up the transform map --> base_link at a different time as the scenario execution node is not allowed to use the sim time
   * - ``namespace_override``
     - ``string``
     - ``''``
     - if set, it's used as namespace (instead of the associated actor's namespace)

``log_check()``
^^^^^^^^^^^^^^^

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

``ros_launch()``
^^^^^^^^^^^^^^^^

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


``ros_run()``
^^^^^^^^^^^^^^^^

Run a package specific executable.

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
     - package that contains the executable
   * - ``executable_name``
     - ``string``
     - 
     - name of executable
   * - ``wait_for_shutdown``
     - ``bool``
     - ``true``
     - If true, the action waits until the execution is finished
   * - ``shutdown_timeout``
     - ``time``
     - ``10s``
     - (Only used ``if wait_for_shutdown`` is ``false``) Time to wait between ``SIGINT`` and ``SIGKILL`` getting sent, if process is still running on scenario shutdown

``service_call()``
^^^^^^^^^^^^^^^^^^

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
^^^^^^^^^^^^^^^^^^^^^^^^

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
^^^^^^^^^^^^^^^^^^^

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
^^^^^^^^^^^^^^^^^^^

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
^^^^^^^^^^^^^^^^^^^

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


``wait_for_nodes()``
^^^^^^^^^^^^^^^^^^^^^

Wait for nodes to get available.

.. list-table:: 
   :widths: 15 15 5 65
   :header-rows: 1
   :class: tight-table   
   
   * - Parameter
     - Type
     - Default
     - Description
   * - ``nodes``
     - ``list of string``
     - 
     - List of nodes to wait for


``wait_for_topics()``
^^^^^^^^^^^^^^^^^^^^^

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


X11
---

The library contains actions to interact with the X11 window system. Import it with ``import osc.x11``. It is provided by the package :repo_link:`libs/scenario_execution_x11`.

``capture_screen()``
^^^^^^^^^^^^^^^^^^^^

Capture the screen content within a video.

.. list-table:: 
   :widths: 15 15 5 65
   :header-rows: 1
   :class: tight-table

   * - Parameter
     - Type
     - Default
     - Description
   * - ``output_filename``
     - ``string``
     - ``capture.mp4``
     - Name of the resulting video file (use ``--output-dir`` command-line argument to store the file within a specific directory)
   * - ``frame_rate``
     - ``float``
     - ``25.0``
     - Frame-rate of the resulting video
