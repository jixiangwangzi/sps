#include "nav2_regulated_pure_pursuit_controller/util/path_process.h"

#include "nav2_regulated_pure_pursuit_controller/util/frame_converter.h"
#include "nav2_regulated_pure_pursuit_controller/util/path_matcher.h"
#include "tf2/utils.h"

namespace util
{
PathProcess::PathProcess(const double interval) : p2p_interval_(interval)
{
    sample_interval_ = 0.1;

    cubic_bezier_ = std::make_unique<CubicBezier>(p2p_interval_);
}

std::vector<CurvePoint> PathProcess::smoothPath(const nav_msgs::msg::Path& path, const double smooth_length)
{
    std::vector<CurvePoint> smooth_path;

    // if (path.poses.empty())
    // {
    //     return smooth_path;
    // }

    // double delta_x = pose.pose.position.x - path.poses.back().pose.position.x;
    // double delta_y = pose.pose.position.y - path.poses.back().pose.position.y;

    // // cubic bezier
    // if (std::hypot(delta_x, delta_y) < 0.5)
    // {
    //     smooth_path = cubic_bezier_->GetInerpolationPoints(
    //         pose.pose.position.x, pose.pose.position.y, tf2::getYaw(pose.pose.orientation),
    //         path.poses.back().pose.position.x, path.poses.back().pose.position.y);

    //     return smooth_path;
    // }
    // cubic spline
    std::vector<float> array_x;
    std::vector<float> array_y;
    double             offset_x        = path.poses.front().pose.position.x;
    double             offset_y        = path.poses.front().pose.position.y;
    double             distance_square = 0.0;

    array_x.emplace_back(path.poses[0].pose.position.x - offset_x);
    array_y.emplace_back(path.poses[0].pose.position.y - offset_y);

    for (size_t index = 1; index < path.poses.size(); ++index)
    {
        double delta_x  = array_x.back() - (path.poses[index].pose.position.x - offset_x);
        double delta_y  = array_y.back() - (path.poses[index].pose.position.y - offset_y);
        distance_square = delta_x * delta_x + delta_y * delta_y;

        if (distance_square > sample_interval_ * sample_interval_)
        {
            array_x.emplace_back(path.poses[index].pose.position.x - offset_x);
            array_y.emplace_back(path.poses[index].pose.position.y - offset_y);
        }

        if (((array_x.size() - 1) * sample_interval_) > smooth_length)
        {
            break;
        }
    }

    cublic_spline::Spline2D spline2d(array_x, array_y);
    common::CurvePoint      path_point;

    for (float i = 0; i < spline2d.s.back(); i += p2p_interval_)
    {
        std::array<float, 2> point = spline2d.calc_postion(i);
        path_point.SetS(static_cast<double>(i));
        path_point.SetX(static_cast<double>(point[0]) + offset_x);
        path_point.SetY(static_cast<double>(point[1]) + offset_y);
        path_point.SetTheta(static_cast<double>(spline2d.calc_yaw(i)));
        path_point.SetKappa(static_cast<double>(spline2d.calc_curvature(i)));

        smooth_path.emplace_back(path_point);
    }

    return smooth_path;
}

std::vector<CurvePoint> PathProcess::getOffsetPath(const std::vector<CurvePoint>& path, const double offset_distance)
{
    std::vector<CurvePoint> offset_path;

    for (auto& pt : path)
    {
        const double rs      = pt.GetS();
        const double rx      = pt.GetX();
        const double ry      = pt.GetY();
        const double rtheta  = pt.GetTheta();
        const double rkappa  = pt.GetKappa();
        const double rdkappa = pt.GetDKappa();

        double x     = 0.0;
        double y     = 0.0;
        double theta = 0.0;
        double kappa = 0.0;
        double v     = 0.0;
        double a     = 0.0;

        std::array<double, 3> s_condition = {rs, 0.0, 0.0};
        std::array<double, 3> l_condition = {offset_distance, 0.0, 0.0};

        FrameConverter::Frenet2Cartesian(rs, rx, ry, rtheta, rkappa, rdkappa, s_condition, l_condition, &x, &y, &theta,
                                         &kappa, &v, &a);

        CurvePoint point;
        point.SetS(rs);
        point.SetX(x);
        point.SetY(y);
        point.SetZ(0.0);
        point.SetTheta(theta);
        point.SetKappa(kappa);

        offset_path.push_back(std::move(point));
    }
    return offset_path;
}

nav_msgs::msg::Path PathProcess::getOffsetPath(const nav_msgs::msg::Path& path, const double offset_distance,
                                               const double extract_max_length)
{
    std::vector<CurvePoint> smooth_path = smoothPath(path, extract_max_length);

    nav_msgs::msg::Path offset_path;
    offset_path.header = path.header;

    for (auto& pt : smooth_path)
    {
        const double rs      = pt.GetS();
        const double rx      = pt.GetX();
        const double ry      = pt.GetY();
        const double rtheta  = pt.GetTheta();
        const double rkappa  = pt.GetKappa();
        const double rdkappa = pt.GetDKappa();

        double x     = 0.0;
        double y     = 0.0;
        double theta = 0.0;
        double kappa = 0.0;
        double v     = 0.0;
        double a     = 0.0;

        std::array<double, 3> s_condition = {rs, 0.0, 0.0};
        std::array<double, 3> l_condition = {offset_distance, 0.0, 0.0};

        FrameConverter::Frenet2Cartesian(rs, rx, ry, rtheta, rkappa, rdkappa, s_condition, l_condition, &x, &y, &theta,
                                         &kappa, &v, &a);

        geometry_msgs::msg::PoseStamped pose;
        pose.header          = path.poses[0].header;
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.position.z = 0.0;
        tf2::Quaternion q;
        q.setRPY(0, 0, theta);
        pose.pose.orientation = tf2::toMsg(q);

        offset_path.poses.emplace_back(pose);
    }

    return offset_path;
}

std::vector<common::CurvePoint> PathProcess::transformGlobalPath(const std::vector<common::CurvePoint>& global_path,
                                                                 const size_t                           start_index,
                                                                 const common::CurvePoint&              pose)
{
    std::vector<common::CurvePoint> transform_path;

    for (size_t index = start_index; index < global_path.size(); ++index)
    {
        auto&  pt      = global_path[index];
        double delta_x = pt.GetX() - pose.GetX();
        double delta_y = pt.GetY() - pose.GetY();

        common::CurvePoint curve_point = pt;
        curve_point.SetX(delta_x * std::cos(pose.GetTheta()) + delta_y * std::sin(pose.GetTheta()));
        curve_point.SetY(delta_y * std::cos(pose.GetTheta()) - delta_x * std::sin(pose.GetTheta()));

        transform_path.emplace_back(curve_point);
    }

    for (size_t index = 0; index < transform_path.size(); ++index)
    {
        double heading_temp = 0.0;
        if (0 == index)
        {
            heading_temp = std::atan2(transform_path[0].GetY(), transform_path[0].GetX());
        }
        else
        {
            heading_temp = std::atan2(transform_path[index].GetY() - transform_path[index - 1].GetY(),
                                      transform_path[index].GetX() - transform_path[index - 1].GetX());
        }
        transform_path[index].SetTheta(heading_temp);
    }
    transform_path.back().SetTheta(transform_path[transform_path.size() - 1].GetTheta());

    return transform_path;
}

std::vector<common::CurvePoint> PathProcess::transformGlobalPath(const std::vector<common::CurvePoint>& global_path,
                                                                 const size_t                           start_index,
                                                                 const geometry_msgs::msg::PoseStamped& pose)
{
    common::CurvePoint robot_pose;
    robot_pose.SetX(pose.pose.position.x);
    robot_pose.SetY(pose.pose.position.y);
    robot_pose.SetTheta(tf2::getYaw(pose.pose.orientation));

    return transformGlobalPath(global_path, start_index, robot_pose);
}

size_t PathProcess::searchNearestPathIndex(const nav_msgs::msg::Path& path, const geometry_msgs::msg::PoseStamped& pose)
{
    // search nearest idx
    double min_distance = std::numeric_limits<double>::max();
    size_t nearest_idx  = 0;

    for (size_t index = 0; index < path.poses.size(); ++index)
    {
        double delta_x         = path.poses[index].pose.position.x - pose.pose.position.x;
        double delta_y         = path.poses[index].pose.position.y - pose.pose.position.y;
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
