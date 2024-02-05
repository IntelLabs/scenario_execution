
Scenario Execution
==================

Scenario Execution for Robotics (SER) is a backend- and middleware-agnostic library, that will enable the robotics community to perform reproducible experiments at scale and allows a seamless transition from simulation to real-world experiments.
SER is written in Python and builds upon the generic scenario description language `OpenScenario2.0 <https://www.asam.net/index.php?eID=dumpFile&t=f&f=3460&token=14e7c7fab9c9b75118bb4939c725738fa0521fe9>`__ and `pytrees <https://py-trees.readthedocs.io/en/devel/>`__.  

SER reads a scenario definition from a file, translates it to a py-trees behavior tree and then executes it. This separation of the scenario definition from the implementation massively reduces the manual efforts of (robotics) scenario creation.
Although SER can be used as a pure Python library, it is mainly targeted to be used with the Robot Operating System (ROS2). The backend-agnostic implementation allows SER to be used with both, robotics simulators such as Gazebo and physical robots, with minimal adaptations necessary in the scenario description file. 

To give an impression of the functionality of scenario execution, the
following animation shows an example scenario with a turtlebot-like
robot in simulation using Nav2 to navigate towards a specified
navigation goal in a simulated warehouse environment. Once the robot
reaches a reference position and a box is spawned in front of the robot
as an unmapped static obstacle that needs to be avoided. Upon arrival of
the goal position, the scenario is ends with success and the simulation
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
   actions
   architecture
   openscenario2_support
   development
