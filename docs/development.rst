Development
===========

Contribute
----------

Before pushing your code, please ensure that the code formatting is
correct by running:

.. code-block:: bash

   make

In case of errors you can run the autoformatter by executing:

.. code-block:: bash

   make format

Testing
-------

To run only specific tests:

.. code-block:: bash

   #using py-test
   colcon build --packages-up-to scenario_execution_ros && reset && pytest-3 -s scenario_execution_ros/test/<TEST>.py 

   #manual run
   colcon build --packages-up-to scenario_execution_ros && reset && ros2 launch scenario_execution_ros scenario_launch.py scenario:=<...> debug:=True


Developing and Debugging with Visual Studio Code
------------------------------------------------

To prevent certain issues, please use the following command for building (remove `/build` and `/install`` if another command was used before).

.. code-block:: bash

   colcon build --symlink-install


In VSCode create new debugging configuration file: Run -> "Add Configuration..."

Add the following entry to the "configurations" element within the previously created `launch.json` file (replace the arguments as required):


.. code-block:: json

           {
               "name": "scenario_execution_ros",
               "type": "python",
               "request": "launch",
               "program": "./install/scenario_execution_ros/lib/scenario_execution_ros/scenario_execution_ros",
               "console": "integratedTerminal",
               "cwd": "${workspaceFolder}",
               "args": ["-l", "TEST_SCENARIO.osc"],
           }

Create an `.env` file by executing:

.. code-block:: bash

   source /opt/ros/humble/setup.bash
   source install/setup.bash
   echo PYTHONPATH=$PYTHONPATH > .env
   echo HOME=$HOME >> .env
   echo AMENT_PREFIX_PATH=$AMENT_PREFIX_PATH >> .env
   echo LD_LIBRARY_PATH=$LD_LIBRARY_PATH >> .env


In vscode, open user settings and enable the following settings:

.. code-block::

   "python.terminal.activateEnvInCurrentTerminal": true


To execute the debug configuration either switch to debug view (on the left) and click on "play" or press F5.


Best known Methods
------------------

Implement an Action
^^^^^^^^^^^^^^^^^^^

- If an action's ``setup()`` fails, raise an exception
- Use a state machine, if multiple steps are required
- Implement a ``shutdown()`` method to cleanup on scenario end.
- For debugging/logging:
   - Make use of ``self.feedback_message``
   - Make use of ``kwargs['logger']``, available in ``setup()``
   - If you want to draw markers for RViz, use ``kwargs['marker_handler']``, available in ``setup()`` (with ROS backend)
- Use arguments from ``__init__()`` for a longer running initialization in ``setup()`` and the arguments from ``execute()`` to set values just before executing the action.
- ``__init__()`` and ``setup()`` are called once, ``execute()`` might be called multiple times.
- osc2 arguments can only be consumed once, either in ``__init__()`` or ``execute()``. Exception: If an ``associated_actor`` exists, it's an argument of both methods.
- Arguments that need late resolving (e.g. referring to variables or external methods) need to be consumed in ``execute()``.
- ``setup()`` provides several arguments that might be useful:
  - ``input_dir``: Directory containing the scenario file
  - ``output_dir``: If given on command-line, contains the directory to save output to
  - ``node``: (``scenario_execution_ros`` only): ROS node to utilize (e.g. create subscribers)
- If your action makes use of variables, set ``resolve_variable_reference_arguments_in_execute`` in ``BaseAction.__init()`` to  ``False``.
  The ``execute()`` method arguments will then contain resolved values as before, except for variable arguments which are accessible
  as ``VariableReference`` (with methods ``set_value()`` and ``get_value()``).

Implement an Action with Complex Behavior Tree
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

For actions that need to provide their own complex behavior tree implementation, inherit from ``BaseActionSubtree`` instead of ``BaseAction``:

- Override ``create_subtree()`` method instead of ``update()``
- ``create_subtree()`` should return a complete ``py_trees.behaviour.Behaviour`` that implements the action logic
- All OSC2 parameters are passed to ``create_subtree()`` method, similar to ``execute()`` in ``BaseAction``
- The subtree is created once during action initialization and managed internally
- Use ``BaseActionSubtree`` when you need composite behaviors, decorators, or complex state machines that are better expressed as behavior trees rather than a single behavior's ``update()`` method
- Arguments follow the same rules as ``BaseAction``: can be consumed in ``__init__()`` or ``create_subtree()``, but not both
