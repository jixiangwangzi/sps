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

#include "nav2_regulated_pure_pursuit_controller/regulated_pure_pursuit_controller.hpp"

#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <limits>
#include <memory>
#include <string>
#include <utility>

#include "nav2_core/exceptions.hpp"
#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/node_utils.hpp"

using nav2_util::declare_parameter_if_not_declared;
using nav2_util::geometry_utils::euclidean_distance;
using std::abs;
using std::hypot;
using std::max;
using std::min;
using namespace nav2_costmap_2d;  // NOLINT

namespace nav2_regulated_pure_pursuit_controller
{

void RegulatedPurePursuitController::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent, std::string name,
                                               const std::shared_ptr<tf2_ros::Buffer>&               tf,
                                               const std::shared_ptr<nav2_costmap_2d::Costmap2DROS>& costmap_ros)
{
    auto node = parent.lock();
    if (!node)
    {
        throw nav2_core::PlannerException("Unable to lock node!");
    }

    costmap_ros_ = costmap_ros;
    costmap_     = costmap_ros_->getCostmap();
    tf_          = tf;
    plugin_name_ = name;
    logger_      = node->get_logger();
    clock_       = node->get_clock();

    double transform_tolerance = 0.1;
    double control_frequency   = 20.0;
    goal_dist_tol_             = 0.25;  // reasonable default before first update

    epsilon_error_            = 0.01;
    check_path_narrow_offset_ = 0.03;
    check_path_wide_offset_   = 0.1;

    declare_parameter_if_not_declared(node, plugin_name_ + ".desired_linear_vel", rclcpp::ParameterValue(0.7));
    declare_parameter_if_not_declared(node, plugin_name_ + ".max_linear_accel", rclcpp::ParameterValue(0.4));  // add
    declare_parameter_if_not_declared(node, plugin_name_ + ".max_linear_decel", rclcpp::ParameterValue(0.4));  // add
    declare_parameter_if_not_declared(node, plugin_name_ + ".lookahead_dist", rclcpp::ParameterValue(0.6));
    declare_parameter_if_not_declared(node, plugin_name_ + ".min_lookahead_dist", rclcpp::ParameterValue(0.3));
    declare_parameter_if_not_declared(node, plugin_name_ + ".max_lookahead_dist", rclcpp::ParameterValue(0.9));
    declare_parameter_if_not_declared(node, plugin_name_ + ".lookahead_time", rclcpp::ParameterValue(1.5));
    declare_parameter_if_not_declared(node, plugin_name_ + ".rotate_to_heading_angular_vel",
                                      rclcpp::ParameterValue(0.4));
    declare_parameter_if_not_declared(node, plugin_name_ + ".transform_tolerance", rclcpp::ParameterValue(0.1));
    declare_parameter_if_not_declared(node, plugin_name_ + ".use_velocity_scaled_lookahead_dist",
                                      rclcpp::ParameterValue(true));
    declare_parameter_if_not_declared(node, plugin_name_ + ".min_approach_linear_velocity",
                                      rclcpp::ParameterValue(0.1));
    declare_parameter_if_not_declared(node, plugin_name_ + ".use_approach_linear_velocity_scaling",
                                      rclcpp::ParameterValue(true));
    declare_parameter_if_not_declared(node, plugin_name_ + ".max_allowed_time_to_collision",
                                      rclcpp::ParameterValue(1.0));
    declare_parameter_if_not_declared(node, plugin_name_ + ".use_regulated_linear_velocity_scaling",
                                      rclcpp::ParameterValue(true));
    declare_parameter_if_not_declared(node, plugin_name_ + ".use_cost_regulated_linear_velocity_scaling",
                                      rclcpp::ParameterValue(true));
    declare_parameter_if_not_declared(node, plugin_name_ + ".cost_scaling_dist", rclcpp::ParameterValue(0.6));
    declare_parameter_if_not_declared(node, plugin_name_ + ".cost_scaling_gain", rclcpp::ParameterValue(1.0));
    declare_parameter_if_not_declared(node, plugin_name_ + ".inflation_cost_scaling_factor",
                                      rclcpp::ParameterValue(3.0));
    declare_parameter_if_not_declared(node, plugin_name_ + ".regulated_linear_scaling_min_radius",
                                      rclcpp::ParameterValue(0.90));
    declare_parameter_if_not_declared(node, plugin_name_ + ".regulated_linear_scaling_min_speed",
                                      rclcpp::ParameterValue(0.1));
    declare_parameter_if_not_declared(node, plugin_name_ + ".use_rotate_to_heading", rclcpp::ParameterValue(true));
    declare_parameter_if_not_declared(node, plugin_name_ + ".rotate_to_heading_min_angle",
                                      rclcpp::ParameterValue(0.785));
    declare_parameter_if_not_declared(node, plugin_name_ + ".rotate_to_heading_max_exit",
                                      rclcpp::ParameterValue(0.087));  // 0.17
    declare_parameter_if_not_declared(node, plugin_name_ + ".max_angular_value", rclcpp::ParameterValue(0.5));
    declare_parameter_if_not_declared(node, plugin_name_ + ".max_angular_accel", rclcpp::ParameterValue(0.4));
    declare_parameter_if_not_declared(node, plugin_name_ + ".min_angular_accel", rclcpp::ParameterValue(0.1));
    declare_parameter_if_not_declared(node, plugin_name_ + ".allow_reversing", rclcpp::ParameterValue(false));
    declare_parameter_if_not_declared(node, plugin_name_ + ".enable_lqr_control", rclcpp::ParameterValue(false));
    declare_parameter_if_not_declared(node, plugin_name_ + ".distance_to_path_end_limit", rclcpp::ParameterValue(1.0));
    declare_parameter_if_not_declared(node, plugin_name_ + ".min_collision_check_distance",
                                      rclcpp::ParameterValue(1.0));
    declare_parameter_if_not_declared(node, plugin_name_ + ".max_collision_check_distance",
                                      rclcpp::ParameterValue(2.0));
    declare_parameter_if_not_declared(node, plugin_name_ + ".wait_obstacle_time", rclcpp::ParameterValue(1.0));
    declare_parameter_if_not_declared(node, plugin_name_ + ".p2p_interval", rclcpp::ParameterValue(0.03));

    node->get_parameter(plugin_name_ + ".desired_linear_vel", desired_linear_vel_);
    base_desired_linear_vel_ = desired_linear_vel_;
    node->get_parameter(plugin_name_ + ".max_linear_accel", max_linear_accel_);  // add
    node->get_parameter(plugin_name_ + ".max_linear_decel", max_linear_decel_);  // add
    node->get_parameter(plugin_name_ + ".lookahead_dist", lookahead_dist_);
    node->get_parameter(plugin_name_ + ".min_lookahead_dist", min_lookahead_dist_);
    node->get_parameter(plugin_name_ + ".max_lookahead_dist", max_lookahead_dist_);
    node->get_parameter(plugin_name_ + ".lookahead_time", lookahead_time_);
    node->get_parameter(plugin_name_ + ".rotate_to_heading_angular_vel", rotate_to_heading_angular_vel_);
    node->get_parameter(plugin_name_ + ".transform_tolerance", transform_tolerance);
    node->get_parameter(plugin_name_ + ".use_velocity_scaled_lookahead_dist", use_velocity_scaled_lookahead_dist_);
    node->get_parameter(plugin_name_ + ".min_approach_linear_velocity", min_approach_linear_velocity_);
    node->get_parameter(plugin_name_ + ".use_approach_linear_velocity_scaling", use_approach_vel_scaling_);
    node->get_parameter(plugin_name_ + ".max_allowed_time_to_collision", max_allowed_time_to_collision_);
    node->get_parameter(plugin_name_ + ".use_regulated_linear_velocity_scaling",
                        use_regulated_linear_velocity_scaling_);
    node->get_parameter(plugin_name_ + ".use_cost_regulated_linear_velocity_scaling",
                        use_cost_regulated_linear_velocity_scaling_);
    node->get_parameter(plugin_name_ + ".cost_scaling_dist", cost_scaling_dist_);
    node->get_parameter(plugin_name_ + ".cost_scaling_gain", cost_scaling_gain_);
    node->get_parameter(plugin_name_ + ".inflation_cost_scaling_factor", inflation_cost_scaling_factor_);
    node->get_parameter(plugin_name_ + ".regulated_linear_scaling_min_radius", regulated_linear_scaling_min_radius_);
    node->get_parameter(plugin_name_ + ".regulated_linear_scaling_min_speed", regulated_linear_scaling_min_speed_);
    node->get_parameter(plugin_name_ + ".use_rotate_to_heading", use_rotate_to_heading_);
    node->get_parameter(plugin_name_ + ".rotate_to_heading_min_angle", rotate_to_heading_min_angle_);
    node->get_parameter(plugin_name_ + ".rotate_to_heading_max_exit", rotate_to_heading_max_exit_);
    node->get_parameter(plugin_name_ + ".max_angular_value", max_angular_value_);
    node->get_parameter(plugin_name_ + ".max_angular_accel", max_angular_accel_);
    node->get_parameter(plugin_name_ + ".min_angular_accel", min_angular_accel_);
    node->get_parameter(plugin_name_ + ".allow_reversing", allow_reversing_);
    node->get_parameter("controller_frequency", control_frequency);
    node->get_parameter(plugin_name_ + ".enable_lqr_control", enable_lqr_control_);
    node->get_parameter(plugin_name_ + ".distance_to_path_end_limit", distance_to_path_end_limit_);
    node->get_parameter(plugin_name_ + ".min_collision_check_distance", min_collision_check_distance_);
    node->get_parameter(plugin_name_ + ".max_collision_check_distance", max_collision_check_distance_);
    node->get_parameter(plugin_name_ + ".wait_obstacle_time", wait_obstacle_time_);
    node->get_parameter(plugin_name_ + ".p2p_interval", p2p_interval_);

    transform_tolerance_ = tf2::durationFromSec(transform_tolerance);
    control_duration_    = 1.0 / control_frequency;

    if (inflation_cost_scaling_factor_ <= 0.0)
    {
        RCLCPP_WARN(logger_, "The value inflation_cost_scaling_factor is incorrectly set, "
                             "it should be >0. Disabling cost regulated linear velocity scaling.");
        use_cost_regulated_linear_velocity_scaling_ = false;
    }

    /** Possible to drive in reverse direction if and only if
     "use_rotate_to_heading" parameter is set to false **/

    if (use_rotate_to_heading_ && allow_reversing_)
    {
        RCLCPP_WARN(logger_, "Disabling reversing. Both use_rotate_to_heading and allow_reversing "
                             "parameter cannot be set to true. By default setting use_rotate_to_heading true");
        allow_reversing_ = false;
    }

    global_path_pub_ = node->create_publisher<nav_msgs::msg::Path>("received_global_plan", 1);
    carrot_pub_      = node->create_publisher<geometry_msgs::msg::PointStamped>("lookahead_point", 1);
    carrot_arc_pub_  = node->create_publisher<nav_msgs::msg::Path>("lookahead_collision_arc", 1);
    collision_pub_   = node->create_publisher<std_msgs::msg::Bool>("collision_flg", 1);
    path_handler_    = std::make_unique<PathHandler>(tf2::durationFromSec(transform_tolerance), tf_, costmap_ros_);

    last_state_        = TRACK_STATE;
    lqr_control_       = std::make_shared<control::LqrControl>(control_duration_);
    collision_checker_ = std::make_unique<CollisionChecker>(node, costmap_ros_ /*, params_*/);
    path_process_      = std::make_unique<PathProcess>(p2p_interval_);
}

void RegulatedPurePursuitController::cleanup()
{
    RCLCPP_INFO(logger_,
                "Cleaning up controller: %s of type"
                " regulated_pure_pursuit_controller::RegulatedPurePursuitController",
                plugin_name_.c_str());
    global_path_pub_.reset();
    carrot_pub_.reset();
    carrot_arc_pub_.reset();
    collision_pub_.reset();
}

void RegulatedPurePursuitController::activate()
{
    RCLCPP_INFO(logger_,
                "Activating controller: %s of type "
                "regulated_pure_pursuit_controller::RegulatedPurePursuitController",
                plugin_name_.c_str());
    global_path_pub_->on_activate();
    carrot_pub_->on_activate();
    carrot_arc_pub_->on_activate();
    collision_pub_->on_activate();
}

void RegulatedPurePursuitController::deactivate()
{
    RCLCPP_INFO(logger_,
                "Deactivating controller: %s of type "
                "regulated_pure_pursuit_controller::RegulatedPurePursuitController",
                plugin_name_.c_str());
    global_path_pub_->on_deactivate();
    carrot_pub_->on_deactivate();
    carrot_arc_pub_->on_deactivate();
    collision_pub_->on_deactivate();
}

std::unique_ptr<geometry_msgs::msg::PointStamped>
RegulatedPurePursuitController::createCarrotMsg(const geometry_msgs::msg::PoseStamped& carrot_pose)
{
    auto carrot_msg     = std::make_unique<geometry_msgs::msg::PointStamped>();
    carrot_msg->header  = carrot_pose.header;
    carrot_msg->point.x = carrot_pose.pose.position.x;
    carrot_msg->point.y = carrot_pose.pose.position.y;
    carrot_msg->point.z = 0.01;  // publish right over map to stand out
    return carrot_msg;
}

double RegulatedPurePursuitController::getLookAheadDistance(const geometry_msgs::msg::Twist& speed)
{
    // If using velocity-scaled look ahead distances, find and clamp the dist
    // Else, use the static look ahead distance
    double lookahead_dist = lookahead_dist_;
    if (use_velocity_scaled_lookahead_dist_)
    {
        lookahead_dist = fabs(speed.linear.x) * lookahead_time_;
        lookahead_dist = std::clamp(lookahead_dist, min_lookahead_dist_, max_lookahead_dist_);
    }

    return lookahead_dist;
}

geometry_msgs::msg::TwistStamped RegulatedPurePursuitController::computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped& pose, const geometry_msgs::msg::Twist& speed,
    nav2_core::GoalChecker* goal_checker)
{
    // Update for the current goal checker's state
    geometry_msgs::msg::Pose  pose_tolerance;
    geometry_msgs::msg::Twist vel_tolerance;
    if (!goal_checker->getTolerances(pose_tolerance, vel_tolerance))
    {
        RCLCPP_WARN(logger_, "Unable to retrieve goal checker's tolerances!");
    }
    else
    {
        goal_dist_tol_ = pose_tolerance.position.x;
    }

    // Transform path to robot base frame
    auto transformed_plan = path_handler_->transformGlobalPlan(pose);

    // Collision checker
    double collision_distance = collision_checker_->calcuCollisionDistanceByPath(global_plan_, pose);

    // Find look ahead distance and point on path and publish
    double lookahead_dist = getLookAheadDistance(speed);

    // Check for reverse driving
    if (allow_reversing_)
    {
        // Cusp check
        double dist_to_direction_change = findDirectionChange(pose);

        // if the lookahead distance is further than the cusp, use the cusp distance instead
        if (dist_to_direction_change < lookahead_dist)
        {
            lookahead_dist = dist_to_direction_change;
        }
    }

    auto carrot_pose = getLookAheadPoint(lookahead_dist, transformed_plan);
    carrot_pub_->publish(createCarrotMsg(carrot_pose));

    double linear_vel, angular_vel;

    // Find distance^2 to look ahead point (carrot) in robot base frame
    // This is the chord length of the circle
    const double carrot_dist2 = (carrot_pose.pose.position.x * carrot_pose.pose.position.x) +
                                (carrot_pose.pose.position.y * carrot_pose.pose.position.y);

    // Find curvature of circle (k = 1 / R)
    double curvature = 0.0;
    if (carrot_dist2 > 0.001)
    {
        curvature = 2.0 * carrot_pose.pose.position.y / carrot_dist2;
    }

    // Setting the velocity direction
    double sign = 1.0;
    if (allow_reversing_)
    {
        sign = carrot_pose.pose.position.x >= 0.0 ? 1.0 : -1.0;
    }

    linear_vel = desired_linear_vel_;

    // Make sure we're in compliance with basic constraints
    double angle_to_heading;

    if (shouldRotateToGoalHeading(carrot_pose))
    {
        double angle_to_goal = tf2::getYaw(transformed_plan.poses.back().pose.orientation);
        rotateToHeading(linear_vel, angular_vel, angle_to_goal, speed);

        last_state_ = ROTATE_STATE;
    }
    else if (shouldRotateToPath(carrot_pose, angle_to_heading) ||
             (last_state_ == ROTATE_STATE && angle_to_heading > rotate_to_heading_max_exit_))
    {
        rotateToHeading(linear_vel, angular_vel, angle_to_heading, speed);

        last_state_ = ROTATE_STATE;
    }
    else
    {
        applyConstraints(fabs(lookahead_dist - sqrt(carrot_dist2)), lookahead_dist, curvature, speed,
                         costAtPose(pose.pose.position.x, pose.pose.position.y), linear_vel, sign);

        // RCLCPP_INFO_THROTTLE(logger_, *(clock_), 500, "applyConstraints ----------- : %f.", linear_vel);

        double distance_to_path_end =
            std::hypot(transformed_plan.poses.back().pose.position.x, transformed_plan.poses.back().pose.position.y);
        double both_side_collision_distance = 100.0;  // calcuCollisionAtBothSidePath(pose);

        double delta_x            = transformed_plan.poses[0].pose.position.x;
        double delta_y            = transformed_plan.poses[0].pose.position.y;
        double lateral_devitation = std::hypot(delta_x, delta_y);

        applyOthersConstraints(lateral_devitation, collision_distance, both_side_collision_distance,
                               distance_to_path_end, speed, linear_vel, sign);

        // RCLCPP_INFO_THROTTLE(logger_, *(clock_), 500, "applyOthersConstraints ----------- : %f.", linear_vel);

        if (enable_lqr_control_)
        {
            angular_vel = lqr_control_->Lqr_control_ros(global_plan_, pose.pose, speed, linear_vel);
        }
        else
        {
            angular_vel = linear_vel * curvature;
        }

        // angular_vel = std::clamp(angular_vel, -1.0 * max_angular_value_, max_angular_value_);

        last_state_ = TRACK_STATE;
    }

    std_msgs::msg::Bool collision_flg;
    collision_flg.data = false;

    // Collision checking on this velocity heading
    if (/*isCollisionImminent(pose, linear_vel, angular_vel) ||*/
        (last_state_ != ROTATE_STATE && linear_vel < 0.001 && angular_vel < 0.001))
    {
        // throw nav2_core::PlannerException("RegulatedPurePursuitController detected collision ahead!");
        collision_flg.data = true;
        collision_pub_->publish(collision_flg);
    }
    collision_pub_->publish(collision_flg);

    // populate and return message
    geometry_msgs::msg::TwistStamped cmd_vel;
    cmd_vel.header          = pose.header;
    cmd_vel.twist.linear.x  = linear_vel;
    cmd_vel.twist.angular.z = angular_vel;

    return cmd_vel;
}

bool RegulatedPurePursuitController::shouldRotateToPath(const geometry_msgs::msg::PoseStamped& carrot_pose,
                                                        double&                                angle_to_path)
{
    // Whether we should rotate robot to rough path heading
    angle_to_path = atan2(carrot_pose.pose.position.y, carrot_pose.pose.position.x);
    return use_rotate_to_heading_ && fabs(angle_to_path) > rotate_to_heading_min_angle_;
}

bool RegulatedPurePursuitController::shouldRotateToGoalHeading(const geometry_msgs::msg::PoseStamped& carrot_pose)
{
    // static bool checked_arrived = false;
    // Whether we should rotate robot to goal heading
    double dist_to_goal = std::hypot(carrot_pose.pose.position.x, carrot_pose.pose.position.y);
    return use_rotate_to_heading_ && dist_to_goal < goal_dist_tol_;

    // if (dist_to_goal > goal_dist_tol_ * 2.0)
    // {
    //     checked_arrived = false;
    // }
    // else if (dist_to_goal < goal_dist_tol_)
    // {
    //     checked_arrived = true;
    // }

    // if (use_rotate_to_heading_ && checked_arrived && dist_to_goal < 1.6 * goal_dist_tol_)
    // {
    //     return true;
    // }

    // return false;
}

void RegulatedPurePursuitController::rotateToHeading(double& linear_vel, double& angular_vel,
                                                     const double&                    angle_to_path,
                                                     const geometry_msgs::msg::Twist& curr_speed)
{
    // Rotate in place using max angular velocity / acceleration possible
    linear_vel        = 0.0;
    const double sign = angle_to_path > 0.0 ? 1.0 : -1.0;
    angular_vel       = sign * rotate_to_heading_angular_vel_;

    const double& dt                         = control_duration_;
    const double  min_feasible_angular_speed = curr_speed.angular.z - max_angular_accel_ * dt;
    const double  max_feasible_angular_speed = curr_speed.angular.z + max_angular_accel_ * dt;
    angular_vel = std::clamp(angular_vel, min_feasible_angular_speed, max_feasible_angular_speed);

    if (sign > 0.0)
    {
        angular_vel = std::max(angular_vel, 0.1);
    }
    else
    {
        angular_vel = std::min(angular_vel, -0.1);
    }
}

geometry_msgs::msg::PoseStamped RegulatedPurePursuitController::getLookAheadPoint(
    const double& lookahead_dist, const nav_msgs::msg::Path& transformed_plan)
{
    // Find the first pose which is at a distance greater than the lookahead distance
    auto goal_pose_it = std::find_if(transformed_plan.poses.begin(), transformed_plan.poses.end(), [&](const auto& ps) {
        return hypot(ps.pose.position.x, ps.pose.position.y) >= lookahead_dist;
    });

    /*double distance_sum = 0;

    for (size_t index = 1; index < transformed_plan.poses.size(); ++index)
    {
        double delta_x = 0.0;
        double delta_y = 0.0;


        delta_x = transformed_plan.poses[index].pose.position.x - transformed_plan.poses[index - 1].pose.position.x;
        delta_y = transformed_plan.poses[index].pose.position.y - transformed_plan.poses[index - 1].pose.position.x;
        distance_sum += std::hypot(delta_x, delta_y);


        if (distance_sum >= lookahead_dist)
        {
            return transformed_plan.poses[index];
        }
    }*/

    // return transformed_plan.poses.back();
    //  If the no pose is not far enough, take the last pose
    if (goal_pose_it == transformed_plan.poses.end())
    {
        goal_pose_it = std::prev(transformed_plan.poses.end());
    }

    return *goal_pose_it;
}

bool RegulatedPurePursuitController::isCollisionImminent(const geometry_msgs::msg::PoseStamped& robot_pose,
                                                         const double& linear_vel, const double& angular_vel)
{
    // Note(stevemacenski): This may be a bit unusual, but the robot_pose is in
    // odom frame and the carrot_pose is in robot base frame.

    // check current point is OK
    if (inCollision(robot_pose.pose.position.x, robot_pose.pose.position.y))
    {
        return true;
    }

    // visualization messages
    nav_msgs::msg::Path arc_pts_msg;
    arc_pts_msg.header.frame_id = costmap_ros_->getGlobalFrameID();
    arc_pts_msg.header.stamp    = robot_pose.header.stamp;
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.frame_id = arc_pts_msg.header.frame_id;
    pose_msg.header.stamp    = arc_pts_msg.header.stamp;

    const double projection_time = costmap_->getResolution() / fabs(linear_vel);

    geometry_msgs::msg::Pose2D curr_pose;
    curr_pose.x     = robot_pose.pose.position.x;
    curr_pose.y     = robot_pose.pose.position.y;
    curr_pose.theta = tf2::getYaw(robot_pose.pose.orientation);

    int i = 1;
    while (true)
    {
        // only forward simulate within time requested
        if (i * projection_time > max_allowed_time_to_collision_)
        {
            break;
        }

        i++;

        // apply velocity at curr_pose over distance
        curr_pose.x += projection_time * (linear_vel * cos(curr_pose.theta));
        curr_pose.y += projection_time * (linear_vel * sin(curr_pose.theta));
        curr_pose.theta += projection_time * angular_vel;

        // store it for visualization
        pose_msg.pose.position.x = curr_pose.x;
        pose_msg.pose.position.y = curr_pose.y;
        pose_msg.pose.position.z = 0.01;
        arc_pts_msg.poses.push_back(pose_msg);

        // check for collision at this point
        if (inCollision(curr_pose.x, curr_pose.y))
        {
            carrot_arc_pub_->publish(arc_pts_msg);
            return true;
        }
    }

    carrot_arc_pub_->publish(arc_pts_msg);

    return false;
}

bool RegulatedPurePursuitController::inCollision(const double& x, const double& y)
{
    unsigned int mx, my;

    if (!costmap_->worldToMap(x, y, mx, my))
    {
        RCLCPP_WARN_THROTTLE(logger_, *(clock_), 30000,
                             "The dimensions of the costmap is too small to successfully check for "
                             "collisions as far ahead as requested. Proceed at your own risk, slow the robot, or "
                             "increase your costmap size.");
        return false;
    }

    unsigned char cost = costmap_->getCost(mx, my);

    if (costmap_ros_->getLayeredCostmap()->isTrackingUnknown())
    {
        return cost >= INSCRIBED_INFLATED_OBSTACLE && cost != NO_INFORMATION;
    }
    else
    {
        return cost >= INSCRIBED_INFLATED_OBSTACLE;
    }
}

double RegulatedPurePursuitController::costAtPose(const double& x, const double& y)
{
    unsigned int mx, my;

    if (!costmap_->worldToMap(x, y, mx, my))
    {
        RCLCPP_FATAL(logger_, "The dimensions of the costmap is too small to fully include your robot's footprint, "
                              "thusly the robot cannot proceed further");
        throw nav2_core::PlannerException("RegulatedPurePursuitController: Dimensions of the costmap are too small "
                                          "to encapsulate the robot footprint at current speeds!");
    }

    unsigned char cost = costmap_->getCost(mx, my);
    return static_cast<double>(cost);
}

void RegulatedPurePursuitController::applyConstraints(const double& dist_error, const double& lookahead_dist,
                                                      const double& curvature,
                                                      const geometry_msgs::msg::Twist& /*curr_speed*/,
                                                      const double& pose_cost, double& linear_vel, double& sign)
{
    double curvature_vel = linear_vel;
    double cost_vel      = linear_vel;
    double approach_vel  = linear_vel;

    // limit the linear velocity by curvature
    const double  radius  = fabs(1.0 / curvature);
    const double& min_rad = regulated_linear_scaling_min_radius_;
    if (use_regulated_linear_velocity_scaling_ && radius < min_rad)
    {
        curvature_vel *= 1.0 - (fabs(radius - min_rad) / min_rad);
    }

    if (curvature > 1.0)
    {
        curvature_vel = std::min(curvature_vel, 0.35);
    }
    else if (curvature > 0.5)
    {
        curvature_vel = std::min(curvature_vel, 0.45);
    }

    // limit the linear velocity by proximity to obstacles
    if (use_cost_regulated_linear_velocity_scaling_ && pose_cost != static_cast<double>(NO_INFORMATION) &&
        pose_cost != static_cast<double>(FREE_SPACE))
    {
        const double inscribed_radius = costmap_ros_->getLayeredCostmap()->getInscribedRadius();
        const double min_distance_to_obstacle =
            (-1.0 / inflation_cost_scaling_factor_) * std::log(pose_cost / (INSCRIBED_INFLATED_OBSTACLE - 1)) +
            inscribed_radius;

        if (min_distance_to_obstacle < cost_scaling_dist_)
        {
            cost_vel *= cost_scaling_gain_ * min_distance_to_obstacle / cost_scaling_dist_;
        }
    }

    // Use the lowest of the 2 constraint heuristics, but above the minimum translational speed
    linear_vel = std::min(cost_vel, curvature_vel);
    linear_vel = std::max(linear_vel, regulated_linear_scaling_min_speed_);
    // std::cout << "curvature vel : " << curvature_vel << ", cost_vel :" << cost_vel << ", linear vel :" << linear_vel
    //           << std::endl;
    // if the actual lookahead distance is shorter than requested, that means we're at the
    // end of the path. We'll scale linear velocity by error to slow to a smooth stop.
    // This expression is eq. to (1) holding time to goal, t, constant using the theoretical
    // lookahead distance and proposed velocity and (2) using t with the actual lookahead
    // distance to scale the velocity (e.g. t = lookahead / velocity, v = carrot / t).
    if (use_approach_vel_scaling_ && dist_error > 2.0 * costmap_->getResolution())
    {
        double velocity_scaling = 1.0 - (dist_error / lookahead_dist);
        double unbounded_vel    = approach_vel * velocity_scaling;
        if (unbounded_vel < min_approach_linear_velocity_)
        {
            approach_vel = min_approach_linear_velocity_;
        }
        else
        {
            approach_vel *= velocity_scaling;
        }

        // Use the lowest velocity between approach and other constraints, if all overlapping
        linear_vel = std::min(linear_vel, approach_vel);
    }

    // Limit linear velocities to be valid
    linear_vel = std::clamp(fabs(linear_vel), 0.0, desired_linear_vel_);
    linear_vel = sign * linear_vel;
}

void RegulatedPurePursuitController::applyOthersConstraints(const double&                    lateral_devitation,
                                                            const double&                    collision_distance,
                                                            const double&                    wide_collision_distance,
                                                            const double&                    distance_to_path_end,
                                                            const geometry_msgs::msg::Twist& speed,
                                                            double& linear_velocity_cmd, double& sign)
{
    static int  check_obstacle_couter = 0;
    static bool obstacle_stoped       = false;

    if (lateral_devitation > 0.1)
    {
        linear_velocity_cmd = std::min(desired_linear_vel_ / 2.0, linear_velocity_cmd);
    }
    else if (lateral_devitation > 0.05)
    {
        linear_velocity_cmd = std::min(desired_linear_vel_ * 2.0 / 3.0, linear_velocity_cmd);
    }

    double collision_vel = linear_velocity_cmd;
    double approach_vel  = linear_velocity_cmd;

    //  limit the linear velocity by arriving the goal
    if (distance_to_path_end < distance_to_path_end_limit_)
    {
        if (distance_to_path_end < 0.2 * distance_to_path_end_limit_)
        {
            approach_vel = min_approach_linear_velocity_;
        }
        else if (distance_to_path_end < 0.4 * distance_to_path_end_limit_)
        {
            approach_vel = 0.2;
        }
        else if (distance_to_path_end < 0.6 * distance_to_path_end_limit_)
        {
            approach_vel = 0.3;
        }
        else if (distance_to_path_end < 0.8 * distance_to_path_end_limit_)
        {
            approach_vel = 0.4;
        }
        else
        {
            approach_vel = 0.5;
        }
    }

    linear_velocity_cmd = std::min(approach_vel, linear_velocity_cmd);

    // limit the linear velocity around path
    if (wide_collision_distance < min_collision_check_distance_)
    {
        linear_velocity_cmd = std::min(desired_linear_vel_ / 2.0, linear_velocity_cmd);
    }
    else if (wide_collision_distance < max_collision_check_distance_)
    {
        linear_velocity_cmd = std::min(desired_linear_vel_ * 2.0 / 3.0, linear_velocity_cmd);
    }
    // limit the linear velocity collision distance
    if (obstacle_stoped && check_obstacle_couter++ > static_cast<int>(wait_obstacle_time_ / control_duration_))
    {
        obstacle_stoped       = false;
        check_obstacle_couter = 0;
    }
    else if (collision_distance <= min_collision_check_distance_)
    {
        linear_velocity_cmd   = 0.0;
        obstacle_stoped       = true;
        check_obstacle_couter = 0;
        RCLCPP_INFO_THROTTLE(logger_, *(clock_), 500, "collision distance : %f", collision_distance);
    }
    else if (collision_distance <= min_collision_check_distance_ * 1.2 && speed.linear.x < 0.01)  // consider obstacle
                                                                                                  // detection error
    {
        linear_velocity_cmd   = 0.0;
        obstacle_stoped       = true;
        check_obstacle_couter = 0;
        // RCLCPP_INFO_THROTTLE(logger_, *(clock_), 500, "collision distance ----------- : %f", collision_distance);
    }
    else if (collision_distance < max_collision_check_distance_)
    {
        collision_vel = 0.5;

        double collision_brake_distance = max_collision_check_distance_ - min_collision_check_distance_;

        if (collision_distance < (min_collision_check_distance_ + 0.2 * collision_brake_distance))
        {
            collision_vel = 0.2;
        }
        else if (collision_distance < (min_collision_check_distance_ + 0.5 * collision_brake_distance))
        {
            collision_vel = 0.4;
        }
        else if (collision_distance < (min_collision_check_distance_ + 0.8 * collision_brake_distance))
        {
            collision_vel = 0.5;
        }
        else
        {
            collision_vel = 0.6;
        }
        // RCLCPP_INFO_THROTTLE(logger_, *(clock_), 200, "collision distance ----------- : %f", collision_distance);
        linear_velocity_cmd = std::min(linear_velocity_cmd, collision_vel);
        linear_velocity_cmd = std::max(min_approach_linear_velocity_, linear_velocity_cmd);
    }
    else
    {
        linear_velocity_cmd = std::max(min_approach_linear_velocity_, linear_velocity_cmd);
    }
    // std::cout << "other constraints : " << approach_vel << "," << collision_vel << "," << linear_velocity_cmd << ","
    //           << speed.linear.x << std::endl;

    if (linear_velocity_cmd > (min_approach_linear_velocity_ - epsilon_error_))
    {
        if (std::fabs(speed.linear.x) > min_approach_linear_velocity_)
        {
            if (std::fabs(speed.linear.x) < (linear_velocity_cmd - 0.05))
            {
                linear_velocity_cmd = speed.linear.x + max_linear_accel_ * control_duration_;
            }
            else if (std::fabs(speed.linear.x) > (linear_velocity_cmd + 0.05))
            {
                linear_velocity_cmd = speed.linear.x - max_linear_decel_ * control_duration_;
            }
        }
        else if (std::fabs(speed.linear.x) < min_approach_linear_velocity_)
        {
            linear_velocity_cmd = 1.5 * min_approach_linear_velocity_;
        }
        linear_velocity_cmd = std::max(linear_velocity_cmd, min_approach_linear_velocity_);
    }

    linear_velocity_cmd = std::clamp(fabs(linear_velocity_cmd), 0.0, desired_linear_vel_);
    linear_velocity_cmd = sign * linear_velocity_cmd;
}

void RegulatedPurePursuitController::setPlan(const nav_msgs::msg::Path& path)
{
    // for test
    // if (global_plan_.poses.empty())
    // {
    //     global_plan_ = path;

    //     path_handler_->setPlan(path);
    // }

    global_plan_ = path;

    path_handler_->setPlan(path);
}

void RegulatedPurePursuitController::setSpeedLimit(const double& speed_limit, const bool& percentage)
{
    if (speed_limit == nav2_costmap_2d::NO_SPEED_LIMIT)
    {
        // Restore default value
        desired_linear_vel_ = base_desired_linear_vel_;
    }
    else
    {
        if (percentage)
        {
            // Speed limit is expressed in % from maximum speed of robot
            desired_linear_vel_ = base_desired_linear_vel_ * speed_limit / 100.0;
        }
        else
        {
            // Speed limit is expressed in absolute value
            desired_linear_vel_ = speed_limit;
        }
    }
}

double RegulatedPurePursuitController::findDirectionChange(const geometry_msgs::msg::PoseStamped& pose)
{
    // Iterating through the global path to determine the position of the cusp
    for (unsigned int pose_id = 1; pose_id < global_plan_.poses.size() - 1; ++pose_id)
    {
        // We have two vectors for the dot product OA and AB. Determining the vectors.
        double oa_x = global_plan_.poses[pose_id].pose.position.x - global_plan_.poses[pose_id - 1].pose.position.x;
        double oa_y = global_plan_.poses[pose_id].pose.position.y - global_plan_.poses[pose_id - 1].pose.position.y;
        double ab_x = global_plan_.poses[pose_id + 1].pose.position.x - global_plan_.poses[pose_id].pose.position.x;
        double ab_y = global_plan_.poses[pose_id + 1].pose.position.y - global_plan_.poses[pose_id].pose.position.y;

        /* Checking for the existance of cusp, in the path, using the dot product
        and determine it's distance from the robot. If there is no cusp in the path,
        then just determine the distance to the goal location. */
        if ((oa_x * ab_x) + (oa_y * ab_y) < 0.0)
        {
            auto x = global_plan_.poses[pose_id].pose.position.x - pose.pose.position.x;
            auto y = global_plan_.poses[pose_id].pose.position.y - pose.pose.position.y;
            return hypot(x, y);  // returning the distance if there is a cusp
        }
    }

    return std::numeric_limits<double>::max();
}

double RegulatedPurePursuitController::calcuCollisionAtPath(const geometry_msgs::msg::PoseStamped& robot_pose,
                                                            const double                           check_path_offset)
{
    double distance_at_global_path = std::numeric_limits<double>::max();

    if (global_plan_.poses.empty())
    {
        return distance_at_global_path;
    }

    for (size_t index = current_path_idx_; index < global_plan_.poses.size(); ++index)
    {
        double delta_x = global_plan_.poses[index].pose.position.x - robot_pose.pose.position.x;
        double delta_y = global_plan_.poses[index].pose.position.y - robot_pose.pose.position.y;
        // check 2m
        if (std::hypot(delta_x, delta_y) > 3.0)
        {
            break;
        }

        double pose_x     = global_plan_.poses[index].pose.position.x;
        double pose_y     = global_plan_.poses[index].pose.position.y;
        double pose_theta = tf2::getYaw(global_plan_.poses[index].pose.orientation);

        double theta_temp = (check_path_offset > 0.0) ? normalizeAngle(pose_theta + M_PI / 2.0) :
                                                        normalizeAngle(pose_theta - M_PI / 2.0);

        double pose_offset_x = pose_x + std::fabs(check_path_offset) * std::cos(theta_temp);
        double pose_offset_y = pose_y + std::fabs(check_path_offset) * std::sin(theta_temp);

        // if (inCollision(pose_x, pose_y))
        if (collision_checker_->inCollision(pose_offset_x, pose_offset_y, pose_theta))
        {
            double delta_x = pose_offset_x - robot_pose.pose.position.x;
            double delta_y = pose_offset_y - robot_pose.pose.position.y;

            return std::hypot(delta_x, delta_y);
        }
    }
    return distance_at_global_path;
}

bool RegulatedPurePursuitController::transformPose(const std::string                      frame,
                                                   const geometry_msgs::msg::PoseStamped& in_pose,
                                                   geometry_msgs::msg::PoseStamped&       out_pose) const
{
    if (in_pose.header.frame_id == frame)
    {
        out_pose = in_pose;
        return true;
    }

    try
    {
        tf_->transform(in_pose, out_pose, frame, transform_tolerance_);
        out_pose.header.frame_id = frame;
        return true;
    }
    catch (tf2::TransformException& ex)
    {
        RCLCPP_ERROR(logger_, "Exception in transformPose: %s", ex.what());
    }
    return false;
}

double RegulatedPurePursuitController::normalizeAngle(const double angle)  //[-M_PI,M_PI]
{
    double a = std::fmod(angle + M_PI, 2.0 * M_PI);

    if (a < 0.0)
    {
        a += (2.0 * M_PI);
    }
    return a - M_PI;
}

bool RegulatedPurePursuitController::searchAroundValidPath(const geometry_msgs::msg::PoseStamped& pose,
                                                           nav_msgs::msg::Path&                   valid_path)
{
    double delta_x = pose.pose.position.x - global_plan_.poses.back().pose.position.x;
    double delta_y = pose.pose.position.y - global_plan_.poses.back().pose.position.y;

    if (std::hypot(delta_x, delta_y) < 1.0)
    {
        return false;
    }

    nav_msgs::msg::Path offset_path;

    for (double l = 0.02; l < 0.1; l += 0.02)
    {
        offset_path.poses.clear();
        offset_path = path_process_->getOffsetPath(global_plan_, l, max_collision_check_distance_);

        if (collision_checker_->calcuCollisionDistanceByPath(offset_path, pose) > 100.0)
        {
            valid_path = offset_path;

            return true;
        }

        offset_path.poses.clear();
        offset_path = path_process_->getOffsetPath(global_plan_, -1.0 * l, max_collision_check_distance_);

        if (collision_checker_->calcuCollisionDistanceByPath(offset_path, pose) > 100.0)
        {
            valid_path = offset_path;
            return true;
        }
    }

    return false;
}

double RegulatedPurePursuitController::calcuCollisionAtBothSidePath(const geometry_msgs::msg::PoseStamped& pose)
{
    double delta_x = pose.pose.position.x - global_plan_.poses.back().pose.position.x;
    double delta_y = pose.pose.position.y - global_plan_.poses.back().pose.position.y;

    if (std::hypot(delta_x, delta_y) < 1.0)
    {
        return false;
    }

    nav_msgs::msg::Path offset_path;

    offset_path.poses.clear();
    offset_path = path_process_->getOffsetPath(global_plan_, 0.1, max_collision_check_distance_);

    double left_side_collision_distance = collision_checker_->calcuCollisionDistanceByPath(offset_path, pose);

    offset_path.poses.clear();
    offset_path = path_process_->getOffsetPath(global_plan_, -0.1, max_collision_check_distance_);

    double right_side_collision_distance = collision_checker_->calcuCollisionDistanceByPath(offset_path, pose);

    return std::min(left_side_collision_distance, right_side_collision_distance);
}

}  // namespace nav2_regulated_pure_pursuit_controller

// Register this controller as a nav2_core plugin
PLUGINLIB_EXPORT_CLASS(nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController, nav2_core::Controller)
