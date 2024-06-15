#ifndef NAV2_LOCALPLANNER__LOCAL_PLANNER_SERVER_HPP_
#define NAV2_LOCALPLANNER__LOCAL_PLANNER_SERVER_HPP_

#include <chrono>
#include <string>
#include <memory>
#include <vector>
#include <unordered_map>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_msgs/action/local_path_plan.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_util/simple_action_server.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/create_timer_ros.h"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_msgs/srv/is_path_valid.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float64.hpp"

namespace nav2_local_planner
{
/**
 * @class nav2_planner::PlannerServer
 * @brief An action server implements the behavior tree's ComputePathToPose
 * interface and hosts various plugins of different algorithms to compute plans.
 */
class LocalPlannerServer : public nav2_util::LifecycleNode
{
public:
  /**
   * @brief A constructor for nav2_planner::PlannerServer
   */
  LocalPlannerServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  /**
   * @brief A destructor for nav2_planner::PlannerServer
   */
  ~LocalPlannerServer();

  /**
   * @brief Method to get plan from the desired plugin
   * @param start starting pose
   * @param goal goal request
   * @return Path
   */
  nav_msgs::msg::Path getPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal);

protected:
  /**
   * @brief Configure member variables and initializes planner
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Activate member variables
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Deactivate member variables
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Reset member variables
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Called when in shutdown state
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

  using ActionLocalPlan = nav2_msgs::action::LocalPathPlan;
  using ActionServerLocalPlan = nav2_util::SimpleActionServer<ActionLocalPlan>;

  /**
   * @brief Check if an action server is valid / active
   * @param action_server Action server to test
   * @return SUCCESS or FAILURE
   */
  template<typename T>
  bool isServerInactive(std::unique_ptr<nav2_util::SimpleActionServer<T>> & action_server);

  /**
   * @brief Check if an action server has a cancellation request pending
   * @param action_server Action server to test
   * @return SUCCESS or FAILURE
   */
  template<typename T>
  bool isCancelRequested(std::unique_ptr<nav2_util::SimpleActionServer<T>> & action_server);

  /**
   * @brief Wait for costmap to be valid with updated sensor data or repopulate after a
   * clearing recovery. Blocks until true without timeout.
   */
  void waitForCostmap();

  /**
   * @brief Check if an action server has a preemption request and replaces the goal
   * with the new preemption goal.
   * @param action_server Action server to get updated goal if required
   * @param goal Goal to overwrite
   */
  template<typename T>
  void getPreemptedGoalIfRequested(
    std::unique_ptr<nav2_util::SimpleActionServer<T>> & action_server,
    typename std::shared_ptr<const typename T::Goal> goal);

  /**
   * @brief Obtain current pose of the robot
   * @param pose To store current pose of the robot
   * @return true if able to obtain current pose of the robot, else false
   */
  bool getRobotPose(geometry_msgs::msg::PoseStamped & pose);

  /**
   * @brief Transform start and goal poses into the costmap
   * global frame for path planning plugins to utilize
   * @param action_server Action server to terminate if required
   * @param start The starting pose to transform
   * @param goal Goal pose to transform
   * @return bool If successful in transforming poses
   */
  template<typename T>
  bool transformPosesToGlobalFrame(
    std::unique_ptr<nav2_util::SimpleActionServer<T>> & action_server,
    const double transform_length,
    const nav_msgs::msg::Path & global_path,
    const double near_to_goal_length,
    geometry_msgs::msg::PoseStamped & curr_start,
    nav_msgs::msg::Path & curr_path,
    bool & near_to_goal);

  /**
   * @brief Validate that the path contains a meaningful path
   * @param action_server Action server to terminate if required
   * @param goal Goal Current goal
   * @param path Current path
   * @param planner_id The planner ID used to generate the path
   * @return bool If path is valid
   */
  template<typename T>
  bool validatePath(
    std::unique_ptr<nav2_util::SimpleActionServer<T>> & action_server,
    const geometry_msgs::msg::PoseStamped & curr_goal,
    const nav_msgs::msg::Path & path);

  // Our action server implements the LocalPathPlan action
  std::unique_ptr<ActionServerLocalPlan> action_server_local_plan_;

  /**
   * @brief The action server callback which calls planner to get the path
   * LocalPathPlan
   */
  void computePlan();

  /**
   * @brief Publish a path for visualization purposes
   * @param path Reference to Global Path
   */
  void publishPlan(
    const rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr& publisher,
    const nav_msgs::msg::Path & path);

  /**
   * @brief Get goal pose for local plan
   * @param path reference path
   * @param goal Goal Pose of local plan
   * @return bool If get goal pose successfully
   */
  bool getGoalPose(
    const nav_msgs::msg::Path & path,
    geometry_msgs::msg::PoseStamped & goal);

  /**
   * @brief Stitch local path and global path
   * @param global_path global path for reference
   * @param local_path local path plan result
   * @param result_path stitch local_path to global_path
   * @return bool If stitch local_path to global_path successfully
  */
  bool stitchGlobalPath(
    const nav_msgs::msg::Path & global_path,
    const nav_msgs::msg::Path & local_path,
    nav_msgs::msg::Path & result_path);

  /**
   * @brief use cubic bezier calculate pose
   * @param p0 first control point of cubic bezier(cubic bezier curve start point)
   * @param p1 second control point of cubic bezier
   * @param p2 third control point of cubic bezier
   * @param p3 final control point of cubic bezier(cubic bezier curve end point)
   * @param t control value of cubic bezier
   * @return current pose of cubic bezier
  */
  geometry_msgs::msg::PoseStamped calcCubicBezier(
    const geometry_msgs::msg::PoseStamped & p0,
    const geometry_msgs::msg::PoseStamped & p1,
    const geometry_msgs::msg::PoseStamped & p2,
    const geometry_msgs::msg::PoseStamped & p3,
    const double t);

    /**
   * @brief The service callback to determine if the path is still valid
   * @param request to the service
   * @param response from the service
   */
  void isPathValid(
    const std::shared_ptr<nav2_msgs::srv::IsPathValid::Request> request,
    std::shared_ptr<nav2_msgs::srv::IsPathValid::Response> response);

  bool isDifferentOfTwoPath(
    const nav_msgs::msg::Path & path1,
    const nav_msgs::msg::Path & path2,
    const double dis_tolerance,
    const double check_dis);

  void naviTypeCallback(const std_msgs::msg::Int32::ConstSharedPtr msg);

  void updateGlobalPath(
    const nav_msgs::msg::Path & last_path,
    const nav_msgs::msg::Path & new_path);

  // Planner
  pluginlib::ClassLoader<nav2_core::GlobalPlanner> gp_loader_;
  std::string planner_id_;
  std::string planner_plugin_;
  nav2_core::GlobalPlanner::Ptr planner_ = nullptr;
  double max_planner_duration_;
  double path_distance_ = 4.0;
  double bezier_transition_threshold_ = 0.523;
  int bezier_move_points_num_ = 10;
  nav_msgs::msg::Path history_path_;
  nav_msgs::msg::Path origin_global_path_;
  //Service is path valid
  rclcpp::Service<nav2_msgs::srv::IsPathValid>::SharedPtr is_path_valid_service_;

  // // Smoother
  // Clock
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
  rclcpp::Clock::SharedPtr clock_;

  // TF buffer
  std::shared_ptr<tf2_ros::Buffer> tf_;

  // Global Costmap
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::unique_ptr<nav2_util::NodeThread> costmap_thread_;
  nav2_costmap_2d::Costmap2D * costmap_;
  std::unique_ptr<nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D*>>
      footprint_collision_checker_;

  // Publishers for the path
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr local_plan_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr plan_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>::SharedPtr obs_publisher_;
};

}  // namespace nav2_local_planner

#endif  // NAV2_LOCALPLANNER__LOCAL_PLANNER_SERVER_HPP_
