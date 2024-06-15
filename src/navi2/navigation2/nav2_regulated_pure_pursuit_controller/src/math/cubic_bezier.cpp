#include "nav2_regulated_pure_pursuit_controller/math/cubic_bezier.h"

#include "nav2_regulated_pure_pursuit_controller/math/math_util.h"

namespace math
{

CubicBezier::CubicBezier(const double sample_interval) : interval_(sample_interval)
{
}

std::vector<common::CurvePoint> CubicBezier::GetInerpolationPoints(const double start_x, const double start_y,
                                                                   const double start_heading, const double end_x,
                                                                   const double end_y)
{
    double       sign        = 1.0;
    const double dx          = end_x - start_x;
    const double dy          = end_y - start_y;
    const double cos_heading = std::cos(start_heading);
    const double sin_heading = std::sin(start_heading);

    double euclidean_distance = std::hypot(dx, dy);
    double vertical_distance  = cos_heading * dy - sin_heading * dx;
    double angle_dev          = std::acos(std::fabs(cos_heading * dx + sin_heading * dy) / euclidean_distance);

    if (vertical_distance < 0)
    {
        sign = -1.0;
    }
    double end_heading = MathUtil::NormalizeAngle(start_heading + 2 * sign * angle_dev);
    return GetInerpolationPoints(start_x, start_y, start_heading, end_x, end_y, end_heading);
}

// p0:start point, p1 and p2 is control points, p3 :end point
std::vector<common::CurvePoint> CubicBezier::GetInerpolationPoints(const double start_x, const double start_y,
                                                                   const double start_heading, const double end_x,
                                                                   const double end_y, const double end_heading)
{
    double distance_start2end = std::hypot(start_x - end_x, start_y - end_y);

    double p1_x, p1_y;
    double p2_x, p2_y;
    ;

    p1_x = start_x + 0.4 * distance_start2end * std::cos(start_heading);
    p1_y = start_x + 0.4 * distance_start2end * std::sin(start_heading);

    p2_x = end_x - 0.4 * distance_start2end * std::cos(end_heading);
    p2_y = end_y - 0.4 * distance_start2end * std::sin(end_heading);

    double x_array[4], y_array[4];
    int    size = static_cast<int>(distance_start2end / interval_);

    common::CurvePoint              point_temp;
    std::vector<common::CurvePoint> xyz_list;

    x_array[0] = start_x;
    y_array[0] = start_y;
    x_array[1] = p1_x;
    y_array[1] = p1_y;
    x_array[2] = p2_x;
    y_array[2] = p2_y;
    x_array[3] = end_x;
    y_array[3] = end_y;

    common::CurvePoint start_pose;
    common::CurvePoint end_pose;
    start_pose.SetX(start_x);
    start_pose.SetY(start_y);
    start_pose.SetTheta(start_heading);
    end_pose.SetX(end_x);
    end_pose.SetY(end_y);
    end_pose.SetTheta(end_heading);

    xyz_list.push_back(start_pose);

    for (int i = 1; i < size; i++)
    {
        double t     = std::min(static_cast<double>(i) * interval_ / distance_start2end, 1.0);
        double pos_x = x_array[0] * (1 - t) * (1 - t) * (1 - t) + 3 * x_array[1] * t * (1 - t) * (1 - t) +
                       3 * x_array[2] * t * t * (1 - t) + x_array[3] * t * t * t;
        double pos_y = y_array[0] * (1 - t) * (1 - t) * (1 - t) + 3 * y_array[1] * t * (1 - t) * (1 - t) +
                       3 * y_array[2] * t * t * (1 - t) + y_array[3] * t * t * t;
        double delta_x = pos_x - xyz_list.back().GetX();
        double delta_y = pos_y - xyz_list.back().GetY();

        point_temp.SetX(pos_x);
        point_temp.SetY(pos_y);
        point_temp.SetTheta(std::atan2(delta_y, delta_x));

        xyz_list.push_back(point_temp);
    }

    xyz_list.push_back(end_pose);

    updatePathCurvature(xyz_list);
    return xyz_list;
}

double CubicBezier::calculateCurvature(common::CurvePoint p1, common::CurvePoint p2, common::CurvePoint p3)
{
    double a = std::hypot(p2.GetX() - p1.GetX(), p2.GetY() - p1.GetY());
    double b = std::hypot(p3.GetX() - p2.GetX(), p3.GetY() - p2.GetY());
    double c = std::hypot(p3.GetX() - p1.GetX(), p3.GetY() - p1.GetY());
    double s = (a + b + c) / 2.0;
    double k = std::sqrt(s * (s - a) * (s - b) * (s - c));

    return 4 * k / (a * b * c);
}

void CubicBezier::updatePathCurvature(std::vector<common::CurvePoint>& path)
{
    if (path.size() < 3)
    {
        return;
    }

    for (size_t index = 1; index < path.size() - 1; ++index)
    {
        // left side or right side
        const double cos_heading = std::cos(path[index - 1].GetTheta());
        const double sin_heading = std::sin(path[index - 1].GetTheta());
        const double dx          = path[index].GetX() - path[index - 1].GetX();
        const double dy          = path[index].GetY() - path[index - 1].GetY();

        double vertical_distance = cos_heading * dy - sin_heading * dx;
        double curvature         = calculateCurvature(path[index - 1], path[index], path[index + 1]);

        path[index].SetKappa(std::copysign(curvature, vertical_distance));
    }

    path[0].SetKappa(path[1].GetKappa());
    path.back().SetKappa(path[path.size() - 2].GetKappa());
}

}  // namespace math
