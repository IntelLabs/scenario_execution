
Setup
=====

Installation from source as ROS 2 workspace
-------------------------------------------

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
