
#ifndef _CURVE_POINT_H_
#define _CURVE_POINT_H_

#include <cmath>
#include <vector>

namespace common
{

class CurvePoint
{
public:
    CurvePoint()  = default;
    ~CurvePoint() = default;

    void   SetX(double x) noexcept;
    double GetX() const noexcept;

    void   SetY(double y) noexcept;
    double GetY() const noexcept;

    void   SetZ(double z) noexcept;
    double GetZ() const noexcept;

    void   SetTheta(double theta) noexcept;
    double GetTheta() const noexcept;

    void   SetKappa(double kappa) noexcept;
    double GetKappa() const noexcept;

    void   SetDKappa(double dkappa) noexcept;
    double GetDKappa() const noexcept;

    void   SetS(double s) noexcept;
    double GetS() const noexcept;

    void   SetV(double v) noexcept;
    double GetV() const noexcept;

    void   SetConf(double conf) noexcept;
    double GetConf() const noexcept;

    double Dist(const CurvePoint& point) const;

protected:
    double x_          = 0.0;
    double y_          = 0.0;
    double z_          = 0.0;
    double theta_      = 0.0;
    double kappa_      = 0.0;
    double dkappa_     = 0.0;
    double s_          = 0.0;
    double v_          = 0.0;
    double confidence_ = 0.0;
};

}  // namespace common

#endif
