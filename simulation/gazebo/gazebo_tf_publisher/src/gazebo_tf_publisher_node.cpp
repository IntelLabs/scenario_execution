//  Copyright (C) 2024 Intel Corporation
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//  http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing,
//  software distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions
//  and limitations under the License.
//
//  SPDX-License-Identifier: Apache-2.0

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <ignition/msgs/pose_v.pb.h>
#include <ignition/transport/Node.hh>
#include <rclcpp/rclcpp.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

class GazeboTFPublisher : public rclcpp::Node {
  std::shared_ptr<ignition::transport::Node> mIgnNode;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr mPublisher;
  std::string gz_pose_topic;
  std::string base_frame_id;
  std::vector<unsigned int> robot_frame_ids;
  std::string previous_frame_id;

  void simulationCallback(const ignition::msgs::Pose_V &poses);

public:
  GazeboTFPublisher();
};

GazeboTFPublisher::GazeboTFPublisher() : Node("gazebo_tf_publisher") {
  mPublisher = this->create_publisher<tf2_msgs::msg::TFMessage>("tf", 10);
  mIgnNode = std::make_shared<ignition::transport::Node>();
  declare_parameter("gz_pose_topic", "/world/name/dynamic_pose/info");
  gz_pose_topic = get_parameter("gz_pose_topic").as_string();
  declare_parameter("base_frame_id", "base_link");
  base_frame_id = get_parameter("base_frame_id").as_string();
  mIgnNode->Subscribe(gz_pose_topic, &GazeboTFPublisher::simulationCallback,
                      this);
}

void GazeboTFPublisher::simulationCallback(
    const ignition::msgs::Pose_V &poses) {
  auto tf_msg = std::make_shared<tf2_msgs::msg::TFMessage>();
  auto ros2_clock = get_clock()->now();
  for (int i = 0; i < poses.pose_size(); i++) {
    if (poses.pose(i).name() == base_frame_id) {
      // each robot_frame_id is expected to have corresponding
      // base_link_id -1 in gazebo pose topic.
      unsigned int robot_frame_id = poses.pose(i).id() - 1;
      if (std::find(robot_frame_ids.begin(), robot_frame_ids.end(),
                    robot_frame_id) == robot_frame_ids.end()) {
        // storing each unique id into robot_frame_ids
        robot_frame_ids.push_back(robot_frame_id);
      }
    }
  }

  // Extracting the ID of each robot from the list of
  // robot frame Ids
  for (unsigned int robot_frame_id : robot_frame_ids) {
    for (int i = 0; i < poses.pose_size(); i++) {
      // Looking for pose with the specific robot_frame_id
      if (poses.pose(i).id() == robot_frame_id) {
        geometry_msgs::msg::TransformStamped tf_frame;
        tf_frame.header.stamp = ros2_clock;
        const ignition::msgs::Pose *pp = &poses.pose(i);
        const ignition::msgs::Vector3d *previous_pv = &pp->position();
        const ignition::msgs::Quaternion *previous_pq = &pp->orientation();
        tf_frame.header.frame_id = "map";
        tf_frame.child_frame_id =
            pp->name() + "_" + base_frame_id +
            "_gt"; // Frame Id convention: robot_name + _base_link + _gt
        tf_frame.transform.translation.x = previous_pv->x();
        tf_frame.transform.translation.y = previous_pv->y();
        tf_frame.transform.translation.z = previous_pv->z();
        tf_frame.transform.rotation.x = previous_pq->x();
        tf_frame.transform.rotation.y = previous_pq->y();
        tf_frame.transform.rotation.z = previous_pq->z();
        tf_frame.transform.rotation.w = previous_pq->w();
        tf_msg->transforms.push_back(tf_frame);
      }
    }
    mPublisher->publish(*tf_msg);
  }
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GazeboTFPublisher>());
  rclcpp::shutdown();

  return 0;
}
