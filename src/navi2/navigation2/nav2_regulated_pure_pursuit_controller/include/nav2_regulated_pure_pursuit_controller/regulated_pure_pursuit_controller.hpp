// Copyright (c) 2020 Shrijit Singh
// Copyright (c) 2020 Samsung Research America
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

#ifndef NAV2_REGULATED_PURE_PURSUIT_CONTROLLER__REGULATED_PURE_PURSUIT_CONTROLLER_HPP_
#define NAV2_REGULATED_PURE_PURSUIT_CONTROLLER__REGULATED_PURE_PURSUIT_CONTROLLER_HPP_

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose2_d.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_regulated_pure_pursuit_controller/controller/lqr_steer_control.h"
#include "nav2_regulated_pure_pursuit_controller/path_handler.hpp"
#include "nav2_regulated_pure_pursuit_controller/util/collision_checker.hpp"
#include "nav2_regulated_pure_pursuit_controller/util/path_process.h"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/odometry_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

using namespace util;
using namespace control;

namespace nav2_regulated_pure_pursuit_controller
{

enum STATE
{
    ROTATE_STATE = 0,
    TRACK_STATE,
};

/**
 * @class nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController
 * @brief Regulated pure pursuit controller plugin
 */
class RegulatedPurePursuitController : public nav2_core::Controller
{
public:
    /**
     * @brief Constructor for nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController
     */
    RegulatedPurePursuitController() = default;

    /**
     * @brief Destrructor for nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController
     */
    ~RegulatedPurePursuitController() override = default;

    /**
     * @brief Configure controller state machine
     * @param parent WeakPtr to node
     * @param name Name of plugin
     * @param tf TF buffer
     * @param costmap_ros Costmap2DROS object of environment
     */
    void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent, std::string name,
                   const std::shared_ptr<tf2_ros::Buffer>&               tf,
                   const std::shared_ptr<nav2_costmap_2d::Costmap2DROS>& costmap_ros) override;

    /**
     * @brief Cleanup controller state machine
     */
    void cleanup() override;

    /**
     * @brief Activate controller state machine
     */
    void activate() override;

    /**
     * @brief Deactivate controller state machine
     */
    void deactivate() override;

    /**
     * @brief Compute the best command given the current pose and velocity, with possible debug information
     *
     * Same as above computeVelocityCommands, but with debug results.
     * If the results pointer is not null, additional information about the twists
     * evaluated will be in results after the call.
     *
     * @param pose      Current robot pose
     * @param velocity  Current robot velocity
     * @param goal_checker   Ptr to the goal checker for this task in case useful in computing commands
     * @return          Best command
     */
    geometry_msgs::msg::TwistStamped computeVelocityCommands(const geometry_msgs::msg::PoseStamped& pose,
                                                             const geometry_msgs::msg::Twist&       velocity,
                                                             nav2_core::GoalChecker* /*goal_checker*/) override;

    /**
     * @brief nav2_core setPlan - Sets the global plan
     * @param path The global plan
     */
    void setPlan(const nav_msgs::msg::Path& path) override;

    /**
     * @brief Limits the maximum linear speed of the robot.
     * @param speed_limit expressed in absolute value (in m/s)
     * or in percentage from maximum robot speed.
     * @param percentage Setting speed limit in percentage if true
     * or in absolute values in false case.
     */
    void setSpeedLimit(const double& speed_limit, const bool& percentage) override;

protected:
    /**
     * @brief      normalize angle to [-PI, PI).
     *
     * @param[in]  angle  The original value of the angle.
     *
     * @return     The normalized value of the angle.
     */
    double normalizeAngle(const double angle);

    /**
     * @brief Transforms global plan into same frame as pose, clips far away poses and possibly prunes passed poses
     * @param pose pose to transform
     * @return Path in new frame
     */
    // nav_msgs::msg::Path transformGlobalPlan(const geometry_msgs::msg::PoseStamped& pose);

    /**
     * @brief Transform a pose to another frame.
     * @param frame Frame ID to transform to
     * @param in_pose Pose input to transform
     * @param out_pose transformed output
     * @return bool if successful
     */
    bool transformPose(const std::string frame, const geometry_msgs::msg::PoseStamped& in_pose,
                       geometry_msgs::msg::PoseStamped& out_pose) const;

    /**
     * @brief Get lookahead distance
     * @param cmd the current speed to use to compute lookahead point
     * @return lookahead distance
     */
    double getLookAheadDistance(const geometry_msgs::msg::Twist&);

    /**
     * @brief Creates a PointStamped message for visualization
     * @param carrot_pose Input carrot point as a PoseStamped
     * @return CarrotMsg a carrot point marker, PointStamped
     */
    std::unique_ptr<geometry_msgs::msg::PointStamped>
    createCarrotMsg(const geometry_msgs::msg::PoseStamped& carrot_pose);

    /**
     * @brief Whether robot should rotate to rough path heading
     * @param carrot_pose current lookahead point
     * @param angle_to_path Angle of robot output relatie to carrot marker
     * @return Whether should rotate to path heading
     */
    bool shouldRotateToPath(const geometry_msgs::msg::PoseStamped& carrot_pose, double& angle_to_path);

    /**
     * @brief Whether robot should rotate to final goal orientation
     * @param carrot_pose current lookahead point
     * @return Whether should rotate to goal heading
     */
    bool shouldRotateToGoalHeading(const geometry_msgs::msg::PoseStamped& carrot_pose);

    /**
     * @brief Create a smooth and kinematically smoothed rotation command
     * @param linear_vel linear velocity
     * @param angular_vel angular velocity
     * @param angle_to_path Angle of robot output relatie to carrot marker
     * @param curr_speed the current robot speed
     */
    void rotateToHeading(double& linear_vel, double& angular_vel, const double& angle_to_path,
                         const geometry_msgs::msg::Twist& curr_speed);

    /**
     * @brief Whether collision is imminent
     * @param robot_pose Pose of robot
     * @param carrot_pose Pose of carrot
     * @param linear_vel linear velocity to forward project
     * @param angular_vel angular velocity to forward project
     * @return Whether collision is imminent
     */
    bool isCollisionImminent(const geometry_msgs::msg::PoseStamped&, const double&, const double&);

    /**
     * @brief Whether point is in collision
     * @param x Pose of pose x
     * @param y Pose of pose y
     * @param theta orientation of Yaw
     * @return Whether in collision
     */
    bool inCollision(const double& x, const double& y);

    /**
     * @brief Cost at a point
     * @param x Pose of pose x
     * @param y Pose of pose y
     * @return Cost of pose in costmap
     */
    double costAtPose(const double& x, const double& y);

    /**
     * @brief apply regulation constraints to the system
     * @param linear_vel robot command linear velocity input
     * @param dist_error error in the carrot distance and lookahead distance
     * @param lookahead_dist optimal lookahead distance
     * @param curvature curvature of path
     * @param speed Speed of robot
     * @param pose_cost cost at this pose
     */
    void applyConstraints(const double& dist_error, const double& lookahead_dist, const double& curvature,
                          const geometry_msgs::msg::Twist& speed, const double& pose_cost, double& linear_vel,
                          double& sign);

    void applyOthersConstraints(const double& lateral_devitation, const double& collision_distance,
                                const double& wide_collision_distance, const double& distance_to_path_end,
                                const geometry_msgs::msg::Twist& speed, double& linear_velocity_cmd, double& sign);

    /**
     * @brief Get lookahead point
     * @param lookahead_dist Optimal lookahead distance
     * @param path Current global path
     * @return Lookahead point
     */
    geometry_msgs::msg::PoseStamped getLookAheadPoint(const double&, const nav_msgs::msg::Path&);

    /**
     * @brief checks for the cusp position
     * @param pose Pose input to determine the cusp position
     * @return robot distance from the cusp
     */
    double findDirectionChange(const geometry_msgs::msg::PoseStamped& pose);

    /**
     * @brief calculate collision distance at track path
     * @param robot_pose the robot current pose
     * @return the distance to obstacle
     */
    // double calcuCollisionAtPath(const geometry_msgs::msg::PoseStamped& robot_pose);

    /**
     * @brief calculate collision distance at track path
     * @param robot_pose the robot current pose
     * @param check_path_offset the check path offset ,left is positive and right is negetive
     * @return the distance to obstacle
     */
    double calcuCollisionAtPath(const geometry_msgs::msg::PoseStamped& robot_pose, const double check_path_offset);

    bool searchAroundValidPath(const geometry_msgs::msg::PoseStamped& pose, nav_msgs::msg::Path& valid_path);

    double calcuCollisionAtBothSidePath(const geometry_msgs::msg::PoseStamped& pose);

    std::shared_ptr<tf2_ros::Buffer>               tf_;
    std::string                                    plugin_name_;
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
    nav2_costmap_2d::Costmap2D*                    costmap_;
    rclcpp::Logger                                 logger_{rclcpp::get_logger("RegulatedPurePursuitController")};
    rclcpp::Clock::SharedPtr                       clock_;

    double        desired_linear_vel_, base_desired_linear_vel_;
    double        max_linear_accel_;  // add
    double        max_linear_decel_;  // add
    double        lookahead_dist_;
    double        rotate_to_heading_angular_vel_;
    double        max_lookahead_dist_;
    double        min_lookahead_dist_;
    double        lookahead_time_;
    bool          use_velocity_scaled_lookahead_dist_;
    tf2::Duration transform_tolerance_;
    bool          use_approach_vel_scaling_;
    double        min_approach_linear_velocity_;
    double        control_duration_;
    double        max_allowed_time_to_collision_;
    bool          use_regulated_linear_velocity_scaling_;
    bool          use_cost_regulated_linear_velocity_scaling_;
    double        cost_scaling_dist_;
    double        cost_scaling_gain_;
    double        inflation_cost_scaling_factor_;
    double        regulated_linear_scaling_min_radius_;
    double        regulated_linear_scaling_min_speed_;
    bool          use_rotate_to_heading_;
    double        max_angular_value_;
    double        max_angular_accel_;
    double        min_angular_accel_;  // 0.1
    double        rotate_to_heading_min_angle_;
    double        rotate_to_heading_max_exit_;  // add
    double        goal_dist_tol_;
    bool          allow_reversing_;

    bool   enable_lqr_control_;
    double distance_to_path_end_limit_;
    double min_collision_check_distance_;
    double max_collision_check_distance_;
    double wait_obstacle_time_;

    double epsilon_error_;
    size_t current_path_idx_;
    double check_path_narrow_offset_;
    double check_path_wide_offset_;
    double p2p_interval_;

    nav_msgs::msg::Path                                                                     global_plan_;
    std::vector<common::CurvePoint>                                                         smooth_path_;
    std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>>              global_path_pub_;
    std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PointStamped>> carrot_pub_;
    std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>>              carrot_arc_pub_;
    std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Bool>>              collision_pub_;
    std::unique_ptr<nav2_regulated_pure_pursuit_controller::PathHandler>                    path_handler_;

    STATE                             last_state_;
    std::shared_ptr<LqrControl>       lqr_control_;
    std::unique_ptr<CollisionChecker> collision_checker_;
    std::unique_ptr<PathProcess>      path_process_;
};

}  // namespace nav2_regulated_pure_pursuit_controller

#endif  // NAV2_REGULATED_PURE_PURSUIT_CONTROLLER__REGULATED_PURE_PURSUIT_CONTROLLER_HPP_
