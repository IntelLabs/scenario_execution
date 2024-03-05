
How to run
==========

First, build the packages:

.. code-block:: bash

   colcon build --packages-up-to scenario_execution_gazebo
   source install/setup.bash

To launch an osc-file with ROS2, use the default launch file:

.. code-block:: bash

   ros2 launch scenario_execution scenario_launch.py scenario:=$(PATH_TO_SCENARIO_FILE) debug:=True log-level:=debug

To run an osc-file with ROS2:

.. code-block:: bash

   ros2 run scenario_execution scenario_execution $(PATH_TO_SCENARIO_FILE)

Use ``-t`` flag to see the printed tree and use ``-d`` flag to see debug
information of py_trees and parser:

.. code-block:: bash

   ros2 run scenario_execution scenario_execution $(PATH_TO_SCENARIO_FILE) -t -d


Using RVIZ to trigger scenario
------------------------------

