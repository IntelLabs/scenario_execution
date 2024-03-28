
Setup
=====

Installation from source as ROS 2 workspace
-------------------------------------------

Prerequisites
^^^^^^^^^^^^^

Install ROS2 humble following the `installation instructions <https://docs.ros.org/en/humble/Installation.html>`_.

Installation
^^^^^^^^^^^^^

Clone the scenario execution repository

.. code-block:: bash

   git clone https://github.com/IntelLabs/scenario_execution.git

and update its submodules by running the following command in the root folder of the cloned repository

.. code-block:: bash

   git submodule update --init

install the necessary dependencies

.. code-block:: bash

   rosdep install  --from-paths . --ignore-src
   pip3 install -r requirements.txt

and build it

.. code-block:: bash

   colcon build

.. _install_with_pip:

Installation with pip as standalone Python package
--------------------------------------------------

:repo_link:`scenario_execution_base` is available as standalone Python package.
To install it using pip, run

.. code-block:: bash

   pip install scenario-execution-base
