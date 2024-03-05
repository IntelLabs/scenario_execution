
Scenario Execution
==================

Scenario Execution for Robotics is a backend- and middleware-agnostic library, that enables the robotics community to perform reproducible experiments at scale and allows a seamless transition from simulation to real-world experiments.
Scenario Execution is written in Python and builds upon the generic scenario description language `OpenScenario2 <https://www.asam.net/static_downloads/public/asam-openscenario/2.0.0/welcome.html>`__ and `pytrees <https://py-trees.readthedocs.io/en/devel/>`__.  

Scenario Execution reads a scenario definition from a file, translates it to a py-trees behavior tree and then executes it. This separation of the scenario definition from the implementation massively reduces the manual efforts of (robotics) scenario creation.
Although Scenario Execution can be used as a pure Python library, it is mainly targeted to be used with the `Robot Operating System (ROS2) <https://www.ros.org/>`__. The backend-agnostic implementation allows Scenario Execution to be used with both, robotics simulators such as `Gazebo <https://gazebosim.org/>`__ and physical robots, with minimal adaptations necessary in the scenario description file. 

To give an impression of the functionality of scenario execution, the
following animation shows an example scenario with a turtlebot-like
robot in simulation using Nav2 to navigate towards a specified
navigation goal in a simulated warehouse environment. Once the robot
reaches a reference position a box is spawned in front of the robot
as an unmapped static obstacle that needs to be avoided. Upon arrival of
the goal position, the scenario ends and the simulation
gets cleaned up.

.. figure:: images/scenario.gif
   :alt: in action

   scenario execution in action

.. toctree::
   :maxdepth: 2
   :caption: Contents:

   setup
   how_to_run
   tutorials
   architecture
   libraries
   development
   openscenario2