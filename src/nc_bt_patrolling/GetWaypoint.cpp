// Copyright 2021 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <string>
#include <iostream>
#include <vector>

#include "nc_bt_patrolling/GetWaypoint.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"

#include "geometry_msgs/msg/pose_stamped.hpp"

#include "rclcpp/rclcpp.hpp"

namespace nc_bt_patrolling
{

int GetWaypoint::current_ = 0;

GetWaypoint::GetWaypoint(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf), Node("nc_bt_patrolling")
{
  rclcpp::Node::SharedPtr node;
  config().blackboard->get("node", node);

  geometry_msgs::msg::PoseStamped wp;
  wp.header.frame_id = "map";
  wp.pose.orientation.w = 1.0;

  // wp1
  declare_parameter("wp_1", std::vector<double>());
  get_parameter("wp_1", wp_);

  wp.pose.position.x = wp_[0];
  wp.pose.position.y = wp_[1];
  waypoints_.push_back(wp);

  // wp2
  declare_parameter("wp_2", std::vector<double>());
  get_parameter("wp_2", wp_);

  wp.pose.position.x = wp_[0];
  wp.pose.position.y = wp_[1];
  waypoints_.push_back(wp);

  // wp3
  declare_parameter("wp_3", std::vector<double>());
  get_parameter("wp_3", wp_);

  wp.pose.position.x = wp_[0];
  wp.pose.position.y = wp_[1];
  waypoints_.push_back(wp);

  // wp4
  declare_parameter("wp_4", std::vector<double>());
  get_parameter("wp_4", wp_);

  wp.pose.position.x = wp_[0];
  wp.pose.position.y = wp_[1];
  waypoints_.push_back(wp);

  // wp5
  declare_parameter("wp_5", std::vector<double>());
  get_parameter("wp_5", wp_);

  wp.pose.position.x = wp_[0];
  wp.pose.position.y = wp_[1];
  waypoints_.push_back(wp);

  // waypoint names
  declare_parameter("wp_names", std::vector<std::string>());
  get_parameter("wp_names", wp_names_);
}

void
GetWaypoint::halt()
{
}

BT::NodeStatus
GetWaypoint::tick()
{
  setOutput("wp_name", wp_names_[current_]);
  setOutput("waypoint", waypoints_[current_++]);
  current_ = current_ % waypoints_.size();

  return BT::NodeStatus::SUCCESS;
}

}  // namespace nc_bt_patrolling

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nc_bt_patrolling::GetWaypoint>("GetWaypoint");
}
