#include "nav2_regulated_pure_pursuit_controller/util/velocity_planner.h"

namespace util
{
VelocityPlanner::VelocityPlanner(const double accelerate, const double decelerate, const double max_v,
                                 const double min_v, const double control_duration)
  : control_duration_(control_duration)
  , accelerate_(accelerate)
  , decelerate_(decelerate)
  , max_velocity_(max_v)
  , min_velocity_(min_v)
{
}

double VelocityPlanner::plan(const double current_v, const double desire_v, std::vector<CurvePoint>& path,
                             const double path_step, const double& obstacle_distance,
                             const double& wide_obstacle_distance, const geometry_msgs::msg::PoseStamped& pose)
{
    double linear_velocity_cmd = desire_v;
    // std::cout << "path size : " << path.size() << "," << path_step << std::endl;
    if (path.empty() || path_step <= 0.0)
    {
        return 0.0;
    }
    // std::cout << "path size : " << path.size() << "," << path_step << std::endl;
    size_t current_idx = searchNearestPathIndex(path, pose.pose.position.x, pose.pose.position.y);

    // limit speed (curvature)
    double curvature_limit_velocity = calculateCurvatureConstraint(path, current_idx, path_step);

    // limit speed (path obstacle)
    double obstacle_limit_velocity = calculateObstacleConstraint(obstacle_distance);

    // limit speed (path side obstacle)
    double narrow_area_limit_velocity = calculateNarrowAreaConstraint(wide_obstacle_distance);

    // the path end set v value min_v
    double remain_distance_limit_velocity = calculateRemainDistanceConstraint((path.size() - current_idx) * path_step);

    linear_velocity_cmd = std::min(linear_velocity_cmd, curvature_limit_velocity);
    linear_velocity_cmd = std::min(linear_velocity_cmd, obstacle_limit_velocity);
    linear_velocity_cmd = std::min(linear_velocity_cmd, narrow_area_limit_velocity);
    linear_velocity_cmd = std::min(linear_velocity_cmd, remain_distance_limit_velocity);

    // std::cout << "limit speed : " << desire_v << "," << curvature_limit_velocity << "," << obstacle_limit_velocity
    //           << "," << narrow_area_limit_velocity << "," << remain_distance_limit_velocity << std::endl;
    if (linear_velocity_cmd > min_velocity_)
    {
        if (current_v > min_velocity_)
        {
            if (current_v < (linear_velocity_cmd - 0.05))
            {
                linear_velocity_cmd = current_v + accelerate_ * control_duration_;
            }
            else if (current_v > (linear_velocity_cmd + 0.05))
            {
                linear_velocity_cmd = current_v - decelerate_ * control_duration_;
            }
            else
            {
                linear_velocity_cmd = current_v;
            }
        }
        else if (current_v < min_velocity_)
        {
            linear_velocity_cmd = min_velocity_;
        }
        linear_velocity_cmd = std::max(linear_velocity_cmd, min_velocity_);
    }

    linear_velocity_cmd = std::clamp(fabs(linear_velocity_cmd), 0.0, max_velocity_);

    return linear_velocity_cmd;
}

void VelocityPlanner::setMaxVelocity(const double max_v)
{
    max_velocity_ = max_v;
}

void VelocityPlanner::setMinVelocity(const double min_v)
{
    min_velocity_ = min_v;
}

double VelocityPlanner::calculateCurvatureConstraint(const std::vector<CurvePoint>& path, const size_t current_idx,
                                                     const double path_step)
{
    double       max_curvature   = 0.0;
    const double search_distance = 2.0;

    for (size_t index = current_idx; index < path.size(); ++index)
    {
        double to_start_distance = (index - current_idx) * path_step;
        if (to_start_distance < search_distance)
        {
            max_curvature = std::max(max_curvature, std::fabs(path[index].GetKappa()));
        }
    }

    if (std::fabs(max_curvature) > curvature_list_[0])
    {
        return curvature_velocity_list_[0];
    }
    else if (std::fabs(max_curvature) > curvature_list_[1])
    {
        return curvature_velocity_list_[1];
    }
    else if (std::fabs(max_curvature) > curvature_list_[2])
    {
        return curvature_velocity_list_[2];
    }
    else if (std::fabs(max_curvature) > curvature_list_[3])
    {
        return curvature_velocity_list_[3];
    }
    else if (std::fabs(max_curvature) > curvature_list_[4])
    {
        return curvature_velocity_list_[4];
    }

    return max_velocity_;
}

double VelocityPlanner::calculateObstacleConstraint(const double& obstacle_distance)
{
    if (obstacle_distance > obstacle_distance_list_[0])
    {
        return obstacle_velocity_list_[0];
    }
    else if (obstacle_distance > obstacle_distance_list_[1])
    {
        return obstacle_velocity_list_[1];
    }
    else if (obstacle_distance > obstacle_distance_list_[2])
    {
        return obstacle_velocity_list_[2];
    }
    else if (obstacle_distance > obstacle_distance_list_[3])
    {
        return obstacle_velocity_list_[3];
    }
    else if (obstacle_distance > obstacle_distance_list_[4])
    {
        return obstacle_velocity_list_[4];
    }
    return 0.0;
}

double VelocityPlanner::calculateNarrowAreaConstraint(const double& wide_collision_distance)
{
    if (wide_collision_distance < min_collision_check_distance_)
    {
        return max_velocity_ / 2.0;
    }
    else if (wide_collision_distance < max_collision_check_distance_)
    {
        return max_velocity_ * 2.0 / 3.0;
    }
    return max_velocity_;
}

double VelocityPlanner::calculateRemainDistanceConstraint(const double& remain_distance)
{
    if (remain_distance > remain_distance_list_[0])
    {
        return remain_velocity_list_[0];
    }
    else if (remain_distance > remain_distance_list_[1])
    {
        return remain_velocity_list_[1];
    }
    else if (remain_distance > remain_distance_list_[2])
    {
        return remain_velocity_list_[2];
    }
    else if (remain_distance > remain_distance_list_[3])
    {
        return remain_velocity_list_[3];
    }
    else if (remain_distance > remain_distance_list_[4])
    {
        return remain_velocity_list_[4];
    }
    return min_velocity_;
}

bool VelocityPlanner::trapeziod_velocity_plan(const double current_v, const double desire_v, const double max_v,
                                              const double min_v, std::vector<CurvePoint>& path,
                                              const size_t current_idx, const double path_step)
{
    for (size_t index = 0; index < path.size(); ++index)
    {
        if (index < current_idx)
        {
            path[index].SetV(current_v);
        }
        else
        {
            path[index].SetV(desire_v);
        }
    }
    size_t end_index        = path.size() - 1;
    double velocity         = 0.0;
    double distance         = 0.0;
    double current_v_square = std::pow(current_v, 2);

    auto speedHelper = [&](bool if_accelearate) {
        auto acceleration = if_accelearate ? accelerate_ : -decelerate_;
        for (size_t i = current_idx; i <= end_index; ++i)
        {
            distance = (i - current_idx + 1) * path_step;
            velocity = std::sqrt(2 * acceleration * distance + current_v_square);
            if (if_accelearate ? velocity <= desire_v : velocity >= desire_v)
            {
                if (velocity > max_v)
                {
                    path[i].SetV(max_v);
                }
                else
                {
                    path[i].SetV(velocity);
                }
            }
            else
            {
                break;
            }
        }
    };
    current_v <= desire_v ? speedHelper(true) : speedHelper(false);

    // plan the velocity of  path end
    for (size_t i = end_index; i > current_idx; --i)
    {
        distance = (end_index - i) * path_step;

        if (distance < reserved_distance_)
        {
            path[i].SetV(min_v);
            continue;
        }

        velocity = std::sqrt(2 * decelerate_ * (distance - reserved_distance_) + min_v * min_v);  // need debug
        if (velocity <= path[i].GetV())
        {
            path[i].SetV(velocity);
        }
        else
        {
            break;
        }
    }

    return true;
}

size_t VelocityPlanner::searchNearestPathIndex(const std::vector<CurvePoint>& path, const double& pose_x,
                                               const double& pose_y)
{
    // search nearest idx
    double min_distance = std::numeric_limits<double>::max();
    size_t nearest_idx  = 0;

    for (size_t index = 0; index < path.size(); ++index)
    {
        double delta_x         = path[index].GetX() - pose_x;
        double delta_y         = path[index].GetY() - pose_y;
        double distance_square = delta_x * delta_x + delta_y * delta_y;

        if (distance_square < min_distance)
        {
            min_distance = distance_square;
            nearest_idx  = index;
        }
    }

    return nearest_idx;
}

}  // namespace util
