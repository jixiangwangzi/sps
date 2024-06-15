#include "nav2_regulated_pure_pursuit_controller/math/math_util.h"

namespace math
{
double MathUtil::NormalizeAngle(const double angle)
{
    double a = std::fmod(angle + M_PI, 2.0 * M_PI);

    if (a < 0.0)
    {
        a += (2.0 * M_PI);
    }
    return a - M_PI;
}

double MathUtil::Slerp(const double a0, const double t0, const double a1, const double t1, const double t)
{
    const double almost_zero = 10 * std::numeric_limits<double>::epsilon();
    if (std::abs(t1 - t0) <= almost_zero)
    {
        std::cout << "input time difference is too small";
        return NormalizeAngle(a0);
    }

    const double a0_n = NormalizeAngle(a0);
    const double a1_n = NormalizeAngle(a1);

    double d = a1_n - a0_n;

    if (d > M_PI)
    {
        d = d - 2 * M_PI;
    }
    else if (d < -M_PI)
    {
        d = d + 2 * M_PI;
    }

    const double r = (t - t0) / (t1 - t0);
    const double a = a0_n + d * r;

    return NormalizeAngle(a);
}

CurvePoint MathUtil::InterpolateUsingLinearApproximation(const CurvePoint& p0, const CurvePoint& p1, const double s)
{
    double s0 = p0.GetS();
    double s1 = p1.GetS();
    if (s0 >= s1)
    {
        std::cout << "interpolate failure for s0 < s1. " << std::endl;
        return CurvePoint();
    }

    double weight = (s - s0) / (s1 - s0);
    double x      = (1 - weight) * p0.GetX() + weight * p1.GetX();
    double y      = (1 - weight) * p0.GetY() + weight * p1.GetY();
    double theta  = Slerp(p0.GetTheta(), p0.GetS(), p1.GetTheta(), p1.GetS(), s);
    double kappa  = (1 - weight) * p0.GetKappa() + weight * p1.GetKappa();
    double dkappa = (1 - weight) * p0.GetDKappa() + weight * p1.GetDKappa();

    CurvePoint path_point;
    path_point.SetX(x);
    path_point.SetY(y);
    path_point.SetZ(0.0);
    path_point.SetTheta(theta);
    path_point.SetKappa(kappa);
    path_point.SetDKappa(dkappa);
    path_point.SetS(s);

    return path_point;
}

double MathUtil::LinearInterpolation(const double x1, const double y1, const double x2, const double y2, const double x)
{
    const double almost_zero = 10 * std::numeric_limits<double>::epsilon();
    if (fabs(x2 - x1) < almost_zero)
    {
        std::cout << "ERROR: x1 = x2" << std::endl;
        return 0.5 * (y1 + y2);
    }
    const double k     = (y2 - y1) / (x2 - x1);
    const double b     = y1 - x1 * k;
    const double y_max = std::max(y1, y2);
    const double y_min = std::min(y1, y2);
    return std::max(y_min, std::min(y_max, k * x + b));
}

}  // namespace math
