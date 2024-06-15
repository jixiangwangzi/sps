#include "nav2_regulated_pure_pursuit_controller/common/curve_point.h"

namespace common
{
void CurvePoint::SetX(double x) noexcept
{
    x_ = x;
}

double CurvePoint::GetX() const noexcept
{
    return x_;
}

void CurvePoint::SetY(double y) noexcept
{
    y_ = y;
}

double CurvePoint::GetY() const noexcept
{
    return y_;
}

void CurvePoint::SetZ(double z) noexcept
{
    z_ = z;
}

double CurvePoint::GetZ() const noexcept
{
    return z_;
}

void CurvePoint::SetTheta(double theta) noexcept
{
    theta_ = theta;
}

double CurvePoint::GetTheta() const noexcept
{
    return theta_;
}

void CurvePoint::SetKappa(double kappa) noexcept
{
    kappa_ = kappa;
}

double CurvePoint::GetKappa() const noexcept
{
    return kappa_;
}

void CurvePoint::SetDKappa(double dkappa) noexcept
{
    dkappa_ = dkappa;
}

double CurvePoint::GetDKappa() const noexcept
{
    return dkappa_;
}

void CurvePoint::SetS(double s) noexcept
{
    s_ = s;
}

double CurvePoint::GetS() const noexcept
{
    return s_;
}

void CurvePoint::SetV(double v) noexcept
{
    v_ = v;
}

double CurvePoint::GetV() const noexcept
{
    return v_;
}

void CurvePoint::SetConf(double conf) noexcept
{
    confidence_ = conf;
}

double CurvePoint::GetConf() const noexcept
{
    return confidence_;
}

double CurvePoint::Dist(const CurvePoint& point) const
{
    double dx = x_ - point.GetX();
    double dy = y_ - point.GetY();
    double dz = z_ - point.GetZ();

    return sqrt(dx * dx + dy * dy + dz * dz);
}

}  // namespace common
