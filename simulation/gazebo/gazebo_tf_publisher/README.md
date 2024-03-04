
# Gazebo TF Publisher

The Gazebo TF Publisher searches for the frame id of the robot and publishes it to the global ROS tf topic if it is started without a namespace. The output Frame id has the following syntax: `robot_name_base_link_gt`. It is expected that the the frames from the gazebo topic are in the following order: `map --> robot_name --> base_link`. Per default `base_frame_id:=base_link`.
To start the gazebo ground truth publisher run the following code with the appropriate gazebo pose topic:
```bash
ros2 launch gazebo_tf_publisher gazebo_tf_publisher_launch.py ign_pose_topic:=/world/name/dynamic_pose/info
```

To view the gazebo topic list run the following command:
```bash
ign topic -l
```
The previous command prints all the gazebo topics and among them the right pose topic and also the name of the robots present in the simulation.

The launch file offers also the possibility to start rviz2 directly with `start_rviz:=True` and to start the node in a specified ROS namespace if desired `use_namespace:=True namespace:=desired_namespace`.
