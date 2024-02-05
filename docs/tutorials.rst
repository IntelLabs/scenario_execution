Tutorials
=========

Table of contents
-----------------

-  `Create a scenario in
   OpenScenario2 <#create-scenario-in-openscenario-v200>`__
-  `Create a custom action plugin <#create-action-plugin>`__
-  `Create a scenario with a simulated Turtlebot4 and
   Nav2 <#create-a-scenario-with-a-simulated-turtlebot4-and-nav2>`__
-  `Create a scenario with a simulated Turtlebot4 and Nav2 and spawning
   a static unmapped
   obstacle <#create-a-scenario-with-a-simulated-turtlebot4-and-nav2-and-spawning-a-static-unmapped-obstacle>`__

Create Scenario in OpenSCENARIO V2.0.0
--------------------------------------

To create a scenario in OpenSCENARIO V2.0.0 syntax, first create a file
with the extension ``.osc``. Input the following code in the file.

::

   # import the libraries with import expression
   import osc.builtin

   # Comments start with "#"
   # This is a comment.

   # declare the scenario by the syntax: "scenario scenario_name:"
   scenario create_scenario_tutorial:
       # define the content of the scenario with "do_directive"
       do serial:
           # log a message on the screen with "log" action from the built-in library
           log() with:
               # constrain the logged msg to "Hello World!"
               keep(it.msg == "Hello World")
           # emit the "end" event to signalize the end of the scenario
           emit end

The first line ``import osc.builtin`` will import the integrated library
``builtin.osc`` from the package “scenario_execution_base”. All the
OpenSCENARIO 2 types defined in the file will be imported into the
parser.

The comments in OpenSCENARIO 2 always start with hashtag “#”.

Then, declare a scenario without an associated actor. The term
“scenario” declares that this is a scenario and
“create_scenario_tutorial” is the name of the scenario followed by a
colon. The content of the scenario is defined in the “do_directive”. The
term ``do`` declares that this is a “do_directive”. The term ``serial``
states that the following layer will be executed in sequence. Use the
``log`` action defined in the built-in library to log a message on the
screen. The term ``with`` declares that there are behavior invocation
members that modify the behavior. In the behavior invocation members, a
“keep constraint” is given to constrain the action ``log``. In side the
“keep constraint”, the ``it`` term references to the current namespace.
In this case, it’s the ``log`` action. With ``.`` symbol, you can access
the members of the ``log`` action. The ``it.msg`` references to the
parameter ``msg`` of the action ``log``. The relational operator ``==``
signalizes that the message should be equal to the string
``"Hello World!"``. After the ``log`` action is invoked, the scenario
should emit an ``end`` event to tell the scenario execution to shut down
and return success on scenario “create_scenario_tutorial”. On the other
hand, if you want to define a condition where the scenario fails, use
``emit fail`` to shut down the scenario execution and return failure.

Use this code to see a launch of this tutorial:

.. code-block:: bash

   colcon build --packages-up-to scenario_execution_tutorials && source install/setup.bash \
   && ros2 launch scenario_execution_tutorials scenario_execution_create_scenario_tutorial_launch.py

Create Action Plugin
--------------------

To show how to create a action plugin for the scenario execution, we
will use the ``log`` action as an example.

First, we need to define the ``tutorial_log`` action in OpenSCENARIO 2.

::

   action tutorial_log:
       plugin: string = "TutorialLog"
       msg: string

The ``plugin`` parameter shows what is the name of the entry point of
the plugin. The ``msg`` parameter is the message we want to log to the
screen.

Then, we can write our code for action plugin in Python.

::

   import py_trees
   from scenario_execution_base.osc2_utils.osc2_type import ActionPlugin

   # Step 1: define the py_trees behavior
   class LogAction(py_trees.behaviour.Behaviour):

       # Override the __init__ function to accept parsed arguments.
       def __init__(self, msg: str):
           super().__init__("Log")
           self.msg = msg

       # Override the update function to define how the behavior is ticking.
       def update(self):
           print(self.msg)
           return py_trees.common.Status.SUCCESS

   # Step 2: create the action plugin by inherit from the class "ActionPlugin"
   class Log(ActionPlugin):
       # Override the init function to give the action plugin a name
       def __init__(self, action_handle) -> None:
           super().__init__('Log', action_handle)

       # override the create_behavior function to return the behavior we just declared.
       def create_behavior(self) -> py_trees.behaviour.Behaviour:
           return LogAction

       # override the parse function to parse the parameter from the osc2 action
       def parse(self):
           """
           Parse the parameters from the action
           """
           self.kwargs['msg'] = self.action_handle.parameters['msg'].value

In the example, we created an action plugin to print a message on the
screen. The first step is to create a py_trees behavior for the action
plugin. First, override the ``__init__()`` function to accept the parsed
parameter from the action plugin. The action plugin ``Log`` only parses
one parameter ``msg``, so the behavior only has to accept ``msg`` as an
argument. Then, override the ``update()`` function to define how the
behavior works. In this case, the behavior print the message on the screen
and then return success.

The second step is to create the action plugin. All action plugins need
to inherit from the class ``ActionPlugin`` from
“scenario_execution_base” package. Then, override the
``create_behavior()`` function to return the py_trees behavior we want
to use in our behavior tree. In this case, we return the ``LogAction``
behavior we just created. Remember to only return the class of the
py_trees behavior, not the instance of the class, because the class
needs to instantiated later when all arguments are parsed. Then, we
override the ``parse()`` function. We define the arguments in the
``self.kwargs`` dictionary, which will be passed to the py_trees
behavior in the ``create_behavior()`` function. In the action plugin, we
have a handle on the ``Action`` object from the parser. We can access
the message parameter through
``self.action_handle.parameters['msg'].value``.

After we wrote the action plugin, we need to add it to the
“scenario_execution.action_plugins” entry points, so that the parser can
find it.

Open up the setup file for your Python package and add this line to the
entry points.

::

   'scenario_execution.action_plugins': [
       'TutorialLog = scenario_execution_tutorial.tutorial_log:Log',
   ],

The setup file should look like this afterwards:

::

   from setuptools import setup

   package_name = 'scenario_execution_tutorials'

   setup(
       name=package_name,
       version='0.0.0',
       packages=[package_name],
       data_files=[
           ('share/ament_index/resource_index/packages',
               ['resource/' + package_name]),
           ('share/' + package_name, ['package.xml']),
       ],
       install_requires=['setuptools'],
       zip_safe=True,
       maintainer='xiyansu',
       maintainer_email='xiyan.su@intel.com',
       description='TODO: Package description',
       license='TODO: License declaration',
       tests_require=['pytest'],
       entry_points={
           'console_scripts': [
           ],
           'scenario_execution.action_plugins': [
               'TutorialLog = scenario_execution_tutorials.tutorial_log:Log',
           ],
       },
   )

Now, you can use your action plugin ``tutorial_log`` in your scenarios:

::

   action tutorial_log:
       plugin: string = "TutorialLog"
       msg: string

   scenario create_action_plugin_tutorial:
       do serial:
           tutorial_log() with:
               keep(it.msg == 'Action Plugin Tutorial')
           emit end

Use this code to see a launch of this tutorial:

.. code-block:: bash

   colcon build --packages-up-to scenario_execution_tutorials && source install/setup.bash \
   && ros2 launch scenario_execution_tutorials scenario_execution_create_action_plugin_tutorial_launch.py

Create a scenario with a simulated Turtlebot4 and Nav2
------------------------------------------------------

A simple example scenario for spawning a simulated Turtlebot4 in Gazebo
and control it with Nav2, can be found in :repo_link:`sscenario_execution_tutorials/scenarios/nav2_simulation_nav_to_pose_tutorial.osc`.

This scenario files looks as follows:

::

   import osc.ros                                                  # imports
   import osc.gazebo

   scenario nav2_simulation_nav_to_pose:                           # scenario name
       turtlebot4: differential_drive_robot with:                  # define turtlebot4 robot
           keep(it.namespace == '')
           keep(it.model == 'topic:///robot_description')
       do parallel:
           test_drive: serial:
               wait_for_sim() with:                                # wait for the simulation to start
                   keep(it.world_name == 'maze')
               turtlebot4.spawn() with:                            # spawn the robot
                   keep(it.spawn_pose.position.x == 0.0m)
                   keep(it.spawn_pose.position.y == 0.0m)
                   keep(it.spawn_pose.position.z == 0.1m)
                   keep(it.spawn_pose.orientation.yaw == 0.0rad)
                   keep(it.world_name == 'maze')
               turtlebot4.init_nav2() with:                        # initialize Nav2
                   keep(it.initial_pose.position.x == 0.0m)
                   keep(it.initial_pose.position.y == 0.0m)
                   keep(it.initial_pose.orientation.yaw == 0.0rad)
               turtlebot4.nav_to_pose() with:                      # navigate to the first goal pose
                   keep(it.goal_pose.position.x == 3.0m)
                   keep(it.goal_pose.position.y == -3.0m)
               turtlebot4.nav_to_pose() with:                      # navigate back to spawning position
                   keep(it.goal_pose.position.x == 0.0m)
                   keep(it.goal_pose.position.y == 0.0m)
               emit end                                            # end the scenario with success
           time_out: serial:                                       # timeout to stop scenario after 4 minutes and mark it as failed in case something goes wrong
               wait elapsed(240s)
               emit fail

Let’s break down the individual components of the scenario. The
following snippet defines the turtlebot4 amr-object and where the
object’s description comes from (in this case, the robot_description
ROS2 topic).

::

   turtlebot4: differential_drive_robot with:                  # define turtlebot4 robot
       keep(it.namespace == '')
       keep(it.model == 'topic:///robot_description')

The ``do parallel`` runs the actual test drive and a time-out in
parallel. In case something goes wrong, the time-out prevents the
scenario from running indefinitely by canceling it after 4 minutes and
marking it as failed.

The following snippet make the scenario execution module wait until the
simulation is up and running. Note, that we need to give the name of the
simulation (here, Ignition) as well as the name of the simulation world
(here, maze) to this action.

::

   wait_for_sim() with:                                # wait for the simulation to start
       keep(it.world_name == 'maze')

The following snippet spawns the robot at the origin of the world/map

::

   turtlebot4.spawn() with:                            # spawn the robot
       keep(it.spawn_pose.position.x == 0.0m)
       keep(it.spawn_pose.position.y == 0.0m)
       keep(it.spawn_pose.position.z == 0.1m)
       keep(it.spawn_pose.orientation.yaw == 0.0rad)
       keep(it.world_name == 'maze')

Before being able to navigate, nav2 needs to be initialized. This
includes setting the initial pose of the Nav2 localization module
`AMCL <https://wiki.ros.org/amcl>`__.

::

   turtlebot4.init_nav2() with:                        # initialize Nav2
       keep(it.initial_pose.position.x == 0.0m)
       keep(it.initial_pose.position.y == 0.0m)
       keep(it.initial_pose.orientation.yaw == 0.0rad)

In case you want to localize your robot with
`SLAM <https://github.com/SteveMacenski/slam_toolbox>`__ instead of
AMCL, there is no initial pose needed, i.e., you need to change the
``init_nav2`` action to

::

   turtlebot4.init_nav2() with:                        # initialize Nav2
       keep(it.initial_pose.position.x == 0.0m)
       keep(it.initial_pose.position.y == 0.0m)
       keep(it.initial_pose.orientation.yaw == 0.0rad)
       keep(it.use_initial_pose == false)              # the initial pose is not needed if we are using slam

Finally, the following snippet calls the Nav2 `NavigateToPose
action <https://github.com/ros-planning/navigation2/blob/main/nav2_msgs/action/NavigateToPose.action>`__
to make the robot navigate to a specified goal pose and back to the
starting position

::

   turtlebot4.nav_to_pose() with:                      # navigate to the first goal pose
       keep(it.goal_pose.position.x == 3.0m)
       keep(it.goal_pose.position.y == -3.0m)
   turtlebot4.nav_to_pose() with:                      # navigate back to spawning position
       keep(it.goal_pose.position.x == 0.0m)
       keep(it.goal_pose.position.y == 0.0m)

Once the robot reached the final goal pose ``emit end`` finishes the
scenario and marks it as successful.

To try this example, run

::

   ros2 launch scenario_execution_tutorials turtlebot4_simulation_nav2_to_pose_tutorial_launch.py headless:=False

and you should see something like this

.. figure:: images/tb4_scenario.gif
   :alt: turtlebot4 nav2 scenario

   Turtlebot4 NAV2 scenario

In case you want to run the navigation with SLAM instead of AMCL, update
the scenario file as described above and then run

::

   ros2 launch scenario_execution_tutorials turtlebot4_simulation_nav2_to_pose_tutorial_launch.py headless:=False slam:=True

and you should see something like this

.. figure:: images/tb4_scenario_slam.PNG
   :alt: turtlebot4 nav2 scenario SLAM

   Turtlebot4 NAV2 scenario SLAM

Create a scenario with a simulated Turtlebot4 and Nav2 and spawning a static unmapped obstacle
----------------------------------------------------------------------------------------------

In this section, we’ll extend the previous example and use the :repo_link:`scenario_execution/action_plugins/tf_close_to.py`.
to spawn a static obstacle in front of the robot once it reaches a
user-specified reference point. The corresponding scenario can be found
in :repo_link:`scenario_execution_tutorials/scenarios/nav2_simulation_nav_to_pose_object_spawn_tutorial.osc <scenarios/nav2_simulation_nav_to_pose_object_spawn_tutorial.osc`.

This scenario only differs from the previous scenario regarding the
definition of the obstacle itself and the condition, when to spawn it.
Here, we’ll only look at the differences to the previous scenario. At
the beginning, we define a box, which will be needed as static obstacle
during the scenario

::

   box: amr_object with:
       keep(it.model == 'scenario_execution_tutorials://models/box.sdf')

Next, we’ll have a look at how to spawn the box when the robot reaches a
certain location. The following scenario snippet shows, how this is
done.

::

   parallel:                                           # navigation task and checking if the robot is close to a position need to happen in parallel
       serial:
           turtlebot4.nav_to_pose() with:              # navigate to the first goal pose
               keep(it.goal_pose.position.x == 3.0m)
               keep(it.goal_pose.position.y == -3.0m)
           turtlebot4.nav_to_pose() with:              # navigate back to spawning position
               keep(it.goal_pose.position.x == 0.0m)
               keep(it.goal_pose.position.y == 0.0m)
       serial:
           turtlebot4.tf_close_to() with:              # check if the robot (ground truth position) is close to a specified reference-point
               keep(it.reference_point.x == 1.5m)
               keep(it.reference_point.y == -1.5m)
               keep(it.threshold == 0.4m)
               keep(it.robot_frame_id == 'turtlebot4_base_link_gt')
           wait elapsed(0.5s)
           box.spawn() with:                           # once the robot reached the reference-point, spawn the box as unmapped static obstacle in the robot's way
               keep(it.spawn_pose.position.x == 2.0m)
               keep(it.spawn_pose.position.y == -2.0m)
               keep(it.spawn_pose.position.z == 0.1m)
               keep(it.spawn_pose.orientation.yaw == 0.0rad)
               keep(it.world_name == 'maze')

First, we wrap the navigation part in the first branch of a parallel
statement. This is necessary, as the condition if the robot reached the
reference-point needs to happen continuously in parallel to the
navigation action. This condition is checked with the ``tf_close_to``
action. Once the robot reaches the reference point, the box is spawned
as unmapped static obstacle in the robot’s way such that the navigation
stack needs to avoid it to reach its goal.

To try this example, run

.. code-block:: bash

   ros2 launch scenario_execution_tutorials turtlebot4_simulation_nav2_to_pose_object_spawn_tutorial_launch.py headless:=False
