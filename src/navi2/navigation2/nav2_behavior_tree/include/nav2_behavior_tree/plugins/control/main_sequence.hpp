/*
* Copyright (c) 2022, Cloudminds, Inc.
* All rights reserved.
*
* author: ShiLiang Li
* Created Date: 2022-10-31
*/

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__CONTROL__MAIN_SEQUENCE_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__CONTROL__MAIN_SEQUENCE_HPP_

#include <string>

#include "behaviortree_cpp_v3/control_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"

namespace nav2_behavior_tree
{

class MainSequence : public BT::ControlNode
{
public:

  explicit MainSequence(const std::string & name);

  MainSequence(const std::string & name, const BT::NodeConfiguration & config);

  BT::NodeStatus tick() override;

  void halt() override;

  static BT::PortsList providedPorts() {return {};}

private:
  bool first_time_ = false;
  unsigned int index_ = 0;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__CONTROL__MAIN_SEQUENCE_HPP_
