#ifndef __CUBIC_BEZIER_H__
#define __CUBIC_BEZIER_H__

#include <cmath>
#include <iostream>
#include <vector>

#include "nav2_regulated_pure_pursuit_controller/common/curve_point.h"

namespace math
{

class CubicBezier
{
public:
    CubicBezier(const double sample_interval);

    std::vector<common::CurvePoint> GetInerpolationPoints(const double start_x, const double start_y,
                                                          const double start_heading, const double end_x,
                                                          const double end_y);
    std::vector<common::CurvePoint> GetInerpolationPoints(const double start_x, const double start_y,
                                                          const double start_heading, const double end_x,
                                                          const double end_y, const double end_heading);

private:
    void updatePathCurvature(std::vector<common::CurvePoint>& path);

    double calculateCurvature(common::CurvePoint p1, common::CurvePoint p2, common::CurvePoint p3);

private:
    double interval_ = 0.03;
};

}  // namespace math

#endif
