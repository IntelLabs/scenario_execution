
Setup
=====

Installation from source as ROS 2 workspace
-------------------------------------------

Clone this repository, update its submodules by running

.. code-block:: bash

   git submodule update --init

install the necessary dependencies

.. code-block:: bash

   rosdep install  --from-paths . --ignore-src
   pip3 install -r requirements.txt

and build it

.. code-block:: bash

   colcon build
