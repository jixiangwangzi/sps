// Copyright (c) 2022 Samsung Research America
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

#ifndef NAV2_REGULATED_PURE_PURSUIT_CONTROLLER__COLLISION_CHECKER_HPP_
#define NAV2_REGULATED_PURE_PURSUIT_CONTROLLER__COLLISION_CHECKER_HPP_

#include <algorithm>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose2_d.hpp"
// #include "nav2_core/controller_exceptions.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
// #include "nav2_regulated_pure_pursuit_controller/parameter_handler.hpp"
#include "nav2_regulated_pure_pursuit_controller/common/curve_point.h"
#include "nav2_regulated_pure_pursuit_controller/common/point.h"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/odometry_utils.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace common;

namespace util
{

/**
 * @class nav2_regulated_pure_pursuit_controller::CollisionChecker
 * @brief Checks for collision based on a RPP control command
 */
class CollisionChecker
{
public:
    /**
     * @brief Constructor for nav2_regulated_pure_pursuit_controller::CollisionChecker
     */
    CollisionChecker(rclcpp_lifecycle::LifecycleNode::SharedPtr     node,
                     std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros /*, Parameters* params*/);

    CollisionChecker() = default;
    /**
     * @brief Destrructor for nav2_regulated_pure_pursuit_controller::CollisionChecker
     */
    ~CollisionChecker() = default;

    /**
     * @brief Whether collision is imminent
     * @param robot_pose Pose of robot
     * @param carrot_pose Pose of carrot
     * @param linear_vel linear velocity to forward project
     * @param angular_vel angular velocity to forward project
     * @param carrot_dist Distance to the carrot for PP
     * @return Whether collision is imminent
     */
    bool isCollisionImminent(const geometry_msgs::msg::PoseStamped&, const double&, const double&, const double&);

    /**
     * @brief checks for collision at projected pose
     * @param x Pose of pose x
     * @param y Pose of pose y
     * @param theta orientation of Yaw
     * @return Whether in collision
     */
    bool inCollision(const double& x, const double& y, const double& theta);

    /**
     * @brief checks for collision at projected pose
     * @param x Pose of pose x
     * @param y Pose of pose y
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
     * @brief calculate collision distance by path
     *
     * @param point the position point
     * @param obstacle_points the obstacle array points
     * @param check_width the check width
     * @return Ture if collison,esle false.
     */
    double calcuCollisionDistanceByPath(const std::vector<CurvePoint>&         path,
                                        const geometry_msgs::msg::PoseStamped& pose);

    /**
     * @brief calculate collision distance by path
     *
     * @param point the position point
     * @param obstacle_points the obstacle array points
     * @param check_width the check width
     * @return Ture if collison,esle false.
     */
    double calcuCollisionDistanceByPath(const nav_msgs::msg::Path& path, const geometry_msgs::msg::PoseStamped& pose);

    /**
     * @brief judge the point to obstacle points distance less check_width
     *
     * @param point the position point
     * @param obstacle_points the obstacle array points
     * @param check_width the check width
     * @return Ture if collison,esle false.
     */
    bool IsCollision(const Point& point, const std::vector<Point>& obstacle_points, const double& check_width);

    /**
     * @brief judge the path to obstacle points distance less check_width
     *
     * @param path the path
     * @param obstacle_points the obstacle array points
     * @param check_width the check width
     * @return Ture if collison,esle false.
     */
    bool IsCollision(const std::vector<CurvePoint>& path, const std::vector<Point>& obstacle_points,
                     const double& check_width);

protected:
    rclcpp::Logger                                 logger_{rclcpp::get_logger("RPPCollisionChecker")};
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
    nav2_costmap_2d::Costmap2D*                    costmap_;
    std::unique_ptr<nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D*>>
        footprint_collision_checker_;
    // Parameters*                                                                params_;
    std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> carrot_arc_pub_;
    rclcpp::Clock::SharedPtr                                                   clock_;
};

}  // namespace util

#endif  // NAV2_REGULATED_PURE_PURSUIT_CONTROLLER__COLLISION_CHECKER_HPP_
