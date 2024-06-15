#ifndef __VELOCITY_PLANNER_H__
#define __VELOCITY_PLANNER_H__
#include <iostream>
#include <limits>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_regulated_pure_pursuit_controller/common/curve_point.h"

using namespace common;

namespace util
{
class VelocityPlanner
{
public:
    VelocityPlanner(const double accelerate, const double decelerate, const double max_v, const double min_v,
                    const double control_duration);
    ~VelocityPlanner() = default;

    double plan(const double current_v, const double desire_v, std::vector<CurvePoint>& path, const double path_step,
                const double& obstacle_distance, const double& wide_obstacle_distance,
                const geometry_msgs::msg::PoseStamped& pose);

    void setMaxVelocity(const double max_v);
    void setMinVelocity(const double min_v);

protected:
    size_t searchNearestPathIndex(const std::vector<CurvePoint>& path, const double& pose_x, const double& pose_y);

    double calculateCurvatureConstraint(const std::vector<CurvePoint>& path, const size_t current_idx,
                                        const double path_step);
    bool trapeziod_velocity_plan(const double current_v, const double desire_v, const double max_v, const double min_v,
                                 std::vector<CurvePoint>& path, const size_t current_idx, const double path_step);

    double calculateObstacleConstraint(const double& obstacle_distance);
    double calculateNarrowAreaConstraint(const double& wide_collision_distance);
    double calculateRemainDistanceConstraint(const double& remain_distance);

private:
    double control_duration_;
    double accelerate_;  ///< max accelerate
    double decelerate_;  ///< max decelerate
    double max_velocity_;
    double min_velocity_;
    double reserved_distance_          = 0.3;  ///< keep min velocity at path end
    double curvature_list_[5]          = {3.0, 2.0, 1.0, 0.5, 0.2};
    double curvature_velocity_list_[5] = {0.1, 0.2, 0.3, 0.4, 0.7};

    double min_collision_check_distance_ = 0.5;
    double max_collision_check_distance_ = 1.5;
    double obstacle_distance_list_[5]    = {1.5, 1.3, 1.1, 0.7, 0.5};
    double obstacle_velocity_list_[5]    = {0.7, 0.6, 0.5, 0.4, 0.2};

    double remain_distance_list_[5] = {1.5, 1.3, 1.1, 0.8, 0.3};
    double remain_velocity_list_[5] = {0.7, 0.6, 0.5, 0.4, 0.2};
};
}  // namespace util

#endif
