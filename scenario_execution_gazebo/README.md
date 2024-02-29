# Scenario Execution Gazebo

The "scenario_execution_gazebo" package is an extension of the existing package "scenario_execution" with Gazebo/AMR dependencies.

Please ensure, that your world loads `libgazebo_ros_state` plugin.

## Behaviors and Action Plugins

There are four action plugins defined in the "scenario_execution_gazebo" package, which are derived from the base classes defined in "scenario_execution_base".

### GazeboActorExists

Action plugin to check if an entity exists in Gazebo.

#### Parameters
- name: string specifying the name of the actor/entity to check for existence
- world_name: string specifying name of the simulation world (default value: "default")

### Gazebo Ground Truth Publisher

The Gazebo Ground Truth Publisher searches for the frame id of the robot and publishes it to the global ROS tf topic if it is started without a namespace. The output Frame id has the following syntax: `robot_name_base_link_gt`. It is expected that the the frames from the gazebo topic are in the following order: `map --> robot_name --> base_link`. Per default `base_frame_id:=base_link`.
To start the gazebo ground truth publisher run the following code with the appropriate ignition pose topic:
```bash
ros2 launch gazebo_tf_publisher gazebo_tf_publisher_launch.py ign_pose_topic:=/world/name/dynamic_pose/info
```

To view the ignition topic list run the following command:
```bash
ign topic -l
```
The previous command prints all the ignition topics and among them the right pose topic and also the name of the robots present in the simulation.

The launch file offers also the possibility to start rviz2 directly with `start_rviz:=True` and to start the node in a specified ROS namespace if desired `use_namespace:=True namespace:=desired_namespace`.

#### Paramters

- reference_point: position_3d specifying the reference point
- threshold: distance threshold, i.e., if the distance of the entity is smaller than the threshold, the Action will return True

### GazeboDeleteActor

Action plugin to delete an entity in Gazebo.


#### Parameters <!-- markdownlint-disable-line no-duplicate-heading -->
- world_name: string specifying name of the simulation world (default value: "default")

### GazeboSpawnActor

Action plugin to spawn an entity in Gazebo.

#### Parameters <!-- markdownlint-disable-line no-duplicate-heading -->
- spawn_pose: 3D pose, specifying the initial pose, i.e., the location where the entity/actor is initially spawned
- world_name: string specifying name of the simulation world (default value: "default")
- xacro_arguments: string specifying a comma-separated list of argument key:=value pairs


### GazeboSpawnMovingActor

Action plugin to spawn a moving actor in Gazebo. Trajectory poses and times for the script tag need to
be defined as parameters

#### Parameters <!-- markdownlint-disable-line no-duplicate-heading -->
- spawn_pose: 3D pose, specifying the initial pose, i.e., the location where the entity/actor is initially spawned
- trajectory: string specifying a comma-separated list of trajectory times $t_{i}$ and poses $p_{i}=(x_{i}, y_{i}, z_{i}, \theta_{i})$ for $i=1, \ldots, n$, where $\theta_{i}$ denotes the yaw angle, i.e., the rotation around the $z$ axis of the actor. The items of the list are separated by whitespaces.
That is, the poses need to be defined as follows: $$t_{1} \ x_{1} \ y_{1} \ z_{1} \ \theta_{1}, t_{2} \ x_{2} \ y_{2} \ z_{2} \ \theta_{2}, \ldots$$ **Note:** in Ignition, the trajectory poses need to be defined relative to the actors initial/spawn_pose whereas in Gazebo classic, the trajectory needs to be defined w.r.t. to the world coordinate system.
- world_name: string specifying name of the simulation world (default value: "default")
- xacro_arguments: string specifying a comma-separated list of argument key:=value pairs
