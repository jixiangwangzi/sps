#include "nav2_regulated_pure_pursuit_controller/lattice_planner/lattice_planner.h"

#include <algorithm>
#include <limits>

#include "nav2_regulated_pure_pursuit_controller/common/curve_point.h"
#include "nav2_regulated_pure_pursuit_controller/lattice_planner/curve_1d_generator.h"
#include "nav2_regulated_pure_pursuit_controller/lattice_planner/curve_evaluator.h"
#include "nav2_regulated_pure_pursuit_controller/lattice_planner/trajectory_generator.h"
#include "nav2_regulated_pure_pursuit_controller/math/spline2d.h"
#include "nav2_regulated_pure_pursuit_controller/util/frame_converter.h"
#include "nav2_regulated_pure_pursuit_controller/util/path_matcher.h"

using namespace cublic_spline;

namespace lattice_planner
{
LatticePlanner::LatticePlanner(std::shared_ptr<CollisionChecker>& collision_checker)
  : collision_checker_(collision_checker)
{
}

bool LatticePlanner::Plan(const std::vector<CurvePoint>& ref_path, const CurvePoint& pose, const double max_width,
                          const double vehicle_width, const double lattice_length,
                          const std::vector<Point>& obstacle_points, std::vector<CurvePoint>& final_path)
{
    // 1. smooth refer path
    std::vector<CurvePoint> smooth_path = ref_path;  // SmoothReferencePath(ref_path, pose);

    if (smooth_path.empty())
    {
        return false;
    }

    // 2.compute initial frenet frame position.
    CurvePoint            ref_matched_point = PathMatcher::Match2Path(smooth_path, pose.GetX(), pose.GetY());
    std::array<double, 3> init_s;
    std::array<double, 3> init_l;
    ComputeInitFrenetState(ref_matched_point, pose, &init_s, &init_l);

    // 3.generate 1d curve set for lateral(s-l).
    std::vector<std::shared_ptr<math::Curve1d>> lat_curve_set;
    Curve1dGenerator curve1d_generator(pose, ref_matched_point, init_s, init_l, smooth_path, obstacle_points);

    curve1d_generator.GenerateCurveSet(max_width, max_width, vehicle_width, lattice_length, lat_curve_set);

    // 4.evaluate 1d curves by cost function then combine lon and lat curve pairs and sort according to cost.
    CurveEvaluator curve_evaluator;
    curve_evaluator.Evaluate(lat_curve_set);

    // 5.generate global path set
    std::vector<std::vector<CurvePoint>> path_set;

    while (curve_evaluator.HasTrajectoryPairs())
    {
        std::vector<CurvePoint>      path;
        CurveEvaluator::PtrCurve1d&& curve_temp = curve_evaluator.PopBestTrajectory();

        TrajectoryGenerator::Generate(smooth_path, curve_temp, init_s[0], path);
        path_set.emplace_back(path);
    }

    // 6. select the optimal path
    if (SelectOptimalPath(path_set, obstacle_points, vehicle_width, final_path))
    {
        return true;
    }
    return false;
}

bool LatticePlanner::Plan(const std::vector<CurvePoint>& ref_path, const geometry_msgs::msg::PoseStamped& pose,
                          const double max_width, const double vehicle_width, const double lattice_length,
                          std::vector<CurvePoint>& final_path)
{
    CurvePoint robot_pose;
    robot_pose.SetX(pose.pose.position.x);
    robot_pose.SetY(pose.pose.position.y);
    robot_pose.SetTheta(tf2::getYaw(pose.pose.orientation));

    // 1. smooth refer path
    std::vector<CurvePoint> smooth_path = ref_path;  // SmoothReferencePath(ref_path, pose);

    if (smooth_path.empty())
    {
        return false;
    }

    // 2.compute initial frenet frame position.
    CurvePoint ref_matched_point = PathMatcher::Match2Path(smooth_path, robot_pose.GetX(), robot_pose.GetY());
    std::array<double, 3> init_s;
    std::array<double, 3> init_l;
    ComputeInitFrenetState(ref_matched_point, robot_pose, &init_s, &init_l);

    // 3.generate 1d curve set for lateral(s-l).
    std::vector<std::shared_ptr<math::Curve1d>> lat_curve_set;
    Curve1dGenerator curve1d_generator(robot_pose, ref_matched_point, init_s, init_l, smooth_path, collision_checker_);

    curve1d_generator.GenerateCurveSet(max_width, max_width, vehicle_width, lattice_length, lat_curve_set);

    // 4.evaluate 1d curves by cost function then combine lon and lat curve pairs and sort according to cost.
    CurveEvaluator curve_evaluator;
    curve_evaluator.Evaluate(lat_curve_set);

    // 5.generate global path set
    std::vector<std::vector<CurvePoint>> path_set;

    while (curve_evaluator.HasTrajectoryPairs())
    {
        std::vector<CurvePoint>      path;
        CurveEvaluator::PtrCurve1d&& curve_temp = curve_evaluator.PopBestTrajectory();

        TrajectoryGenerator::Generate(smooth_path, curve_temp, init_s[0], path);
        path_set.emplace_back(path);
    }

    // 6. select the optimal path
    if (SelectOptimalPath(path_set, final_path))
    {
        return true;
    }
    return false;
}

bool LatticePlanner::SelectOptimalPath(const std::vector<std::vector<CurvePoint>>& path_set,
                                       std::vector<CurvePoint>&                    final_path)
{
    for (auto& path : path_set)
    {
        bool is_collision = false;
        for (auto& pt : path)
        {
            if (collision_checker_ && collision_checker_->inCollision(pt.GetX(), pt.GetY(), pt.GetTheta()))
            {
                is_collision = true;
                break;
            }
        }
        if (!is_collision)
        {
            final_path = path;
            return true;
        }
    }
    return false;
}

bool LatticePlanner::SelectOptimalPath(const std::vector<std::vector<CurvePoint>>& path_set,
                                       const std::vector<Point>& obstacle_points, const double& vehicle_width,
                                       std::vector<CurvePoint>& final_path)
{
    for (auto& path : path_set)
    {
        if (!collision_checker_ || !collision_checker_->IsCollision(path, obstacle_points, vehicle_width / 2.0))
        {
            final_path = path;
            return true;
        }
    }
    return false;
}

void LatticePlanner::ComputeInitFrenetState(const CurvePoint& matched_point, const CurvePoint& pose,
                                            std::array<double, 3>* ptr_s, std::array<double, 3>* ptr_d)
{
    FrameConverter::Cartesian2Frenet(matched_point.GetS(), matched_point.GetX(), matched_point.GetY(),
                                     matched_point.GetTheta(), matched_point.GetKappa(), matched_point.GetDKappa(),
                                     pose.GetX(), pose.GetY(), 0, 0, pose.GetTheta(), 0, ptr_s, ptr_d);
}

std::vector<CurvePoint> LatticePlanner::SmoothReferencePath(const std::vector<CurvePoint>& refer_path,
                                                            const CurvePoint&              current_pose)
{
    std::vector<CurvePoint> final_path;
    std::vector<float>      x_array;
    std::vector<float>      y_array;

    // Find the closest pose to current pose on global path
    auto find_closest_pose_idx = [&current_pose, &refer_path]() {
        size_t closest_pose_idx = 0;
        double curr_min_dist    = std::numeric_limits<double>::max();
        for (size_t curr_idx = 0; curr_idx < refer_path.size(); ++curr_idx)
        {
            double curr_dist = std::hypot(current_pose.GetX() - refer_path[curr_idx].GetX(),
                                          current_pose.GetY() - refer_path[curr_idx].GetY());
            if (curr_dist < curr_min_dist)
            {
                curr_min_dist    = curr_dist;
                closest_pose_idx = curr_idx;
            }
        }
        return closest_pose_idx;
    };

    size_t closest_pose_id = find_closest_pose_idx();

    closest_pose_id = closest_pose_id > 2 ? (closest_pose_id - 2) : closest_pose_id;
    x_array.emplace_back(refer_path[closest_pose_id].GetX());
    y_array.emplace_back(refer_path[closest_pose_id].GetY());

    for (size_t cur_idx = closest_pose_id + 1; cur_idx < refer_path.size(); ++cur_idx)
    {
        double interval_distance =
            std::hypot(x_array.back() - refer_path[cur_idx].GetX(), y_array.back() - refer_path[cur_idx].GetY());
        if (interval_distance >= 0.5)
        {
            x_array.emplace_back(refer_path[cur_idx].GetX());
            y_array.emplace_back(refer_path[cur_idx].GetY());
        }
        if (x_array.size() > 8)  // greater 4m
        {
            break;
        }
    }

    if (x_array.size() < 3)
    {
        return final_path;
    }

    Spline2D                spline2d(x_array, y_array);
    std::vector<CurvePoint> path_points;
    CurvePoint              path_point;

    for (float i = 0; i < spline2d.s.back(); i += 0.05)
    {
        std::array<float, 2> point = spline2d.calc_postion(i);
        path_point.SetS(i);
        path_point.SetX(point[0]);
        path_point.SetY(point[1]);
        path_point.SetTheta(spline2d.calc_yaw(i));
        path_point.SetKappa(spline2d.calc_curvature(i));
        path_points.emplace_back(path_point);
    }

    return path_points;
}
}  // namespace lattice_planner
