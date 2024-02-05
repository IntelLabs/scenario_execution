
Development
===========

Contribute
----------

Before pushing your code, please ensure that the code formatting is
correct by running:

::

   make

In case of errors you can run the autoformatter by executing:

::

   make format

Testing
-------

To run only specific tests:

::

   #using py-test
   colcon build --packages-up-to scenario_execution && reset && pytest-3 -s scenario_execution/test/<TEST>.py 

   #manual run
   colcon build --packages-up-to scenario_execution && reset && ros2 launch scenario_execution scenario_launch.py scenario:=<...> debug:=True


Developing and Debugging with Visual Studio Code
------------------------------------------------

To prevent certain issues, please use the following command for building (remove /build and /install if another command was used before).

```
colcon build --symlink-install
```

In VSCode create new debugging configuration file: Run -> "Add Configuration..."

Add the following entry to the "configurations" element within the previously created `launch.json` file (replace the arguments as required):

::

           {
               "name": "scneario_execution",
               "type": "python",
               "request": "launch",
               "program": "./install/scenario_execution/lib/scenario_execution/scenario_execution",
               "console": "integratedTerminal",
               "cwd": "${workspaceFolder}",
               "args": ["-o", "TEST_SCENARIO.osc"],
           }


To execute the debug configuration either switch to debug view (on the left) and click on "play" or press F5.
  
On the first run, there will be errors because of missing dependencies. Within the terminal showing the errors, run:

::

   source /opt/ros/humble/setup.bash
   source install/setup.bash


Afterwards, press F5 again and the execution should succeed. You can now add breakpoints etc.


Creating an Action
------------------

-  If an action setup() should fail, raise an exception
-  Use a state machine, if multiple steps are required
-  Implement a ``cleanup`` method to cleanup on scenario end.
