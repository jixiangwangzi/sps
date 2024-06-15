#include "nav2_regulated_pure_pursuit_controller/controller/pure_pursuit_control.h"

namespace control
{
PurePursuitControl::PurePursuitControl(const double& min_lookahead_distance, const double& max_lookahead_distance)
  : min_lookahead_distance_(min_lookahead_distance), max_lookahead_distance_(max_lookahead_distance)
{
    path_process_ = std::make_unique<PathProcess>();
}

double PurePursuitControl::CalcuControlCmd(const std::vector<CurvePoint>&         global_path,
                                           const geometry_msgs::msg::PoseStamped& cur_pose, const double& cur_velocity,
                                           const double& desire_velocity, const double pre_time)
{
    size_t nearest_idx = searchNearestIndex(global_path, cur_pose);

    // 1. transform the robot coodinate
    auto transformed_plan = path_process_->transformGlobalPath(global_path, nearest_idx, cur_pose);

    /// 2. calculate look ahead distance by speed
    double lookahead_dist = calculateLookAheadDistance(cur_velocity, pre_time);

    /// 3.search the look ahead point
    common::CurvePoint lookahead_point = searchLookAheadPoint(transformed_plan, lookahead_dist);

    /// 4. calculate curvature from current robot position to lookahead point
    const double to_goal_distance = std::hypot(lookahead_point.GetX(), lookahead_point.GetY());
    double       curvature        = 0.0;

    if (to_goal_distance > 0.001)
    {
        curvature = 2.0 * lookahead_point.GetY() / (to_goal_distance * to_goal_distance);
    }

    double angular_velocity_cmd = desire_velocity * curvature;

    return angular_velocity_cmd;
}

CurvePoint PurePursuitControl::getLookAheadPoint(const std::vector<CurvePoint>&         global_path,
                                                 const geometry_msgs::msg::PoseStamped& cur_pose,
                                                 const double& cur_velocity, const double pre_time)
{
    size_t nearest_idx = searchNearestIndex(global_path, cur_pose);
    // 1. transform the robot coodinate
    auto transformed_plan = path_process_->transformGlobalPath(global_path, nearest_idx, cur_pose);

    /// 2. calculate look ahead distance by speed
    double lookahead_dist = calculateLookAheadDistance(cur_velocity, pre_time);

    /// 3.search the look ahead point
    common::CurvePoint lookahead_point = searchLookAheadPoint(transformed_plan, lookahead_dist);

    return lookahead_point;
}

double PurePursuitControl::CalcuControlCmd(const CurvePoint& lookahead_point, const double& desire_velocity)
{
    /// 4. calculate curvature from current robot position to lookahead point
    const double to_goal_distance = std::hypot(lookahead_point.GetX(), lookahead_point.GetY());
    double       curvature        = 0.0;

    if (to_goal_distance > 0.001)
    {
        curvature = 2.0 * lookahead_point.GetY() / (to_goal_distance * to_goal_distance);
    }

    double angular_velocity_cmd = desire_velocity * curvature;

    return angular_velocity_cmd;
}

double PurePursuitControl::calculateLookAheadDistance(const double& linear_velocity, const double& pre_time)
{
    double lookahead_dist = linear_velocity * pre_time;
    lookahead_dist        = std::clamp(lookahead_dist, min_lookahead_distance_, max_lookahead_distance_);

    return lookahead_dist;
}

common::CurvePoint PurePursuitControl::searchLookAheadPoint(const std::vector<common::CurvePoint>& robot_path,
                                                            const double                           lookahead_distance)
{
    auto goal_pose_it = std::find_if(robot_path.begin(), robot_path.end(),
                                     [&](const auto& ps) { return hypot(ps.GetX(), ps.GetY()) >= lookahead_distance; });

    // If the no pose is not far enough, take the last pose
    if (goal_pose_it == robot_path.end())
    {
        goal_pose_it = std::prev(robot_path.end());
    }

    return *goal_pose_it;

    // double distance_sum = 0;

    // for (size_t index = 1; index < transformed_plan.poses.size(); ++index)
    // {
    //     double delta_x = 0.0;
    //     double delta_y = 0.0;

    //     delta_x = transformed_plan.poses[index].pose.position.x - transformed_plan.poses[index - 1].pose.position.x;
    //     delta_y = transformed_plan.poses[index].pose.position.y - transformed_plan.poses[index - 1].pose.position.x;
    //     distance_sum += std::hypot(delta_x, delta_y);

    //     if (distance_sum >= lookahead_distance)
    //     {
    //         return transformed_plan.poses[index];
    //     }
    // }

    // return transformed_plan.poses.back();
}

size_t PurePursuitControl::searchNearestIndex(const std::vector<CurvePoint>&         global_path,
                                              const geometry_msgs::msg::PoseStamped& pose)
{
    double min_distance = std::numeric_limits<double>::max();
    size_t nearest_idx  = 0;

    for (size_t index = 0; index < global_path.size(); ++index)
    {
        double delta_x         = global_path[index].GetX() - pose.pose.position.x;
        double delta_y         = global_path[index].GetY() - pose.pose.position.y;
        double distance_square = delta_x * delta_x + delta_y * delta_y;

        if (distance_square < min_distance)
        {
            min_distance = distance_square;
            nearest_idx  = index;
        }
    }

    return nearest_idx;
}

}  // namespace control
