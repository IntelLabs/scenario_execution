
Setup
=====

Installation with ROS 2
-----------------------

Prerequisites
^^^^^^^^^^^^^

Install ROS2 following the `installation instructions <https://docs.ros.org/en/jazzy/Installation.html>`_ for your distribution `$ROS_DISTRO`.

Scenario execution currently supports the ROS 2 distributions `Humble <https://docs.ros.org/en/humble/index.html>`_ and `Jazzy <https://docs.ros.org/en/jazzy/index.html>`_.

Installation as Debian package (recommended)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

To install scenario execution together with all its libraries, run

.. code-block:: bash

   sudo apt update && sudo apt install -y ros-$ROS_DISTRO-scenario_execution*

To install just the core packages of scenario execution, run

.. code-block:: bash

   sudo apt update && sudo apt install -y ros-$ROS_DISTRO-scenario_execution ros-$ROS_DISTRO-scenario_execution_ros ros-$ROS_DISTRO-scenario_execution_rviz  


Developer Installation (from source as ROS 2 workspace)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Clone the scenario execution repository

.. code-block:: bash

   git clone https://github.com/IntelLabs/scenario_execution.git

and install the necessary dependencies

.. code-block:: bash

   rosdep install  --from-paths . --ignore-src
   pip3 install -r requirements.txt

Now, build your workspace by running

.. code-block:: bash

   colcon build

and source your installation by running

.. code-block:: bash

   source /opt/ros/$ROS_DISTRO/setup.bash && source install/setup.bash

.. _install_with_pip:

Installation with pip as standalone Python package
--------------------------------------------------

:repo_link:`scenario_execution` is available as standalone Python package.
To install it using pip, run

.. code-block:: bash

   pip install scenario-execution
