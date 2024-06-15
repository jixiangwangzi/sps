#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__LOCAL_PATH_PLAN_ACTION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__LOCAL_PATH_PLAN_ACTION_HPP_

#include <string>

#include "nav2_msgs/action/local_path_plan.hpp"
#include "nav_msgs/msg/path.h"
#include "nav2_behavior_tree/bt_action_node.hpp"

namespace nav2_behavior_tree
{

/**
 * @brief A nav2_behavior_tree::BtActionNode class that wraps nav2_msgs::action::SmoothPath
 */
class LocalPathPlanAction : public nav2_behavior_tree::BtActionNode<nav2_msgs::action::LocalPathPlan>
{
public:
  /**
   * @brief A constructor for nav2_behavior_tree::LocalPathPlanAction
   * @param xml_tag_name Name for the XML tag for this node
   * @param action_name Action name this node creates a client for
   * @param conf BT node configuration
   */
  LocalPathPlanAction(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief Function to perform some user-defined operation on tick
   */
  void on_tick() override;

  /**
   * @brief Function to perform some user-defined operation upon successful completion of the action
   */
  BT::NodeStatus on_success() override;

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::OutputPort<nav_msgs::msg::Path>(
          "result_path",
          "Path created by LocalPlannerServer node"),
        BT::OutputPort<double>("planning_time", "Time taken to local path plan"),
        BT::InputPort<nav_msgs::msg::Path>("global_path", "Global path"),
        BT::InputPort<double>("truncate_distance", "Length of Truncated global path to be reference of local path plan"),
        BT::InputPort<double>("near_to_goal_length", "length of path to judge if robot near to goal"),
      });
  }
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__SMOOTH_PATH_ACTION_HPP_
