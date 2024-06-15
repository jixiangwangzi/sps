#include "nav2_regulated_pure_pursuit_controller/util/path_matcher.h"

#include <algorithm>

#include "nav2_regulated_pure_pursuit_controller/math/math_util.h"

using namespace math;

namespace util
{
CurvePoint PathMatcher::Match2Path(const std::vector<CurvePoint>& reference_line, const double x, const double y)
{
    std::function<double(CurvePoint const&, double const, double const)> func_distance_square =
        [](CurvePoint const& pp, double const x, double const y) {
            double dx = pp.GetX() - x;
            double dy = pp.GetY() - y;
            return dx * dx + dy * dy;
        };
    // search nearest point index
    double      distance_min = func_distance_square(reference_line.front(), x, y);
    std::size_t index_min    = 0;

    for (std::size_t i = 1; i < reference_line.size(); ++i)
    {
        double distance_temp = func_distance_square(reference_line[i], x, y);
        if (distance_temp < distance_min)
        {
            distance_min = distance_temp;
            index_min    = i;
        }
    }

    std::size_t index_start = (index_min == 0) ? index_min : index_min - 1;
    std::size_t index_end   = (index_min + 1 == reference_line.size()) ? index_min : index_min + 1;

    if (index_start == index_end)
    {
        return reference_line[index_start];
    }

    return FindProjectionPoint(reference_line[index_start], reference_line[index_end], x, y);
}

CurvePoint PathMatcher::Match2Path(const std::vector<CurvePoint>& reference_line, const double s)
{
    std::function<bool(CurvePoint const&, double const)> comp = [](CurvePoint const& point, double const s) {
        return point.GetS() < s;
    };

    std::vector<CurvePoint>::const_iterator it_lower = std::lower_bound(
        reference_line.begin(), reference_line.end(), s, comp);  // return the first greater the value s iterator
    if (it_lower == reference_line.begin())
    {
        return reference_line.front();
    }
    else if (it_lower == reference_line.end())
    {
        return reference_line.back();
    }

    // interpolate between it_lower - 1 and it_lower
    return MathUtil::InterpolateUsingLinearApproximation(*(it_lower - 1), *it_lower, s);  //使用线性近似进行插值
}

std::pair<double, double> PathMatcher::GetPathFrenetCoordinate(const std::vector<CurvePoint>& reference_line,
                                                               const double x, const double y)
{
    CurvePoint                matched_path_point = Match2Path(reference_line, x, y);
    double                    rtheta             = matched_path_point.GetTheta();
    double                    rx                 = matched_path_point.GetX();
    double                    ry                 = matched_path_point.GetY();
    double                    delta_x            = x - rx;
    double                    delta_y            = y - ry;
    double                    side               = std::cos(rtheta) * delta_y - std::sin(rtheta) * delta_x;
    std::pair<double, double> relative_coordinate;
    relative_coordinate.first  = matched_path_point.GetS();
    relative_coordinate.second = std::copysign(std::hypot(delta_x, delta_y), side);
    return relative_coordinate;
}

CurvePoint  // a.b = |a||b|cost(theta)    a.b/|a| = |b|cost(theta)
PathMatcher::FindProjectionPoint(const CurvePoint& p0, const CurvePoint& p1, const double x, const double y)
{
    double v0x = x - p0.GetX();
    double v0y = y - p0.GetY();

    double v1x = p1.GetX() - p0.GetX();
    double v1y = p1.GetY() - p0.GetY();

    double v1_norm = std::sqrt(v1x * v1x + v1y * v1y);
    double dot     = v0x * v1x + v0y * v1y;

    double delta_s = dot / v1_norm;
    return MathUtil::InterpolateUsingLinearApproximation(p0, p1, p0.GetS() + delta_s);
}

}  // namespace util
