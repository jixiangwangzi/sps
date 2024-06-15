
#ifndef __QUINTIC_POLYNOMIAL_CURVE_1D_H_
#define __QUINTIC_POLYNOMIAL_CURVE_1D_H_

#include <array>

#include "nav2_regulated_pure_pursuit_controller/math/curve_1d.h"

namespace math
{
/**
 * @brief      QuinticPolynomialCurve1d class
 *
 *             1-dimensional quintic polynomial curve.
 */
class QuinticPolynomialCurve1d : public Curve1d
{
public:
    QuinticPolynomialCurve1d()
    {
    }

    QuinticPolynomialCurve1d(const std::array<double, 3>& start, const std::array<double, 3>& end, const double param);

    QuinticPolynomialCurve1d(const double x0, const double dx0, const double ddx0, const double x1, const double dx1,
                             const double ddx1, const double param);

    QuinticPolynomialCurve1d(const QuinticPolynomialCurve1d& other);

    virtual ~QuinticPolynomialCurve1d() = default;

    virtual double Evaluate(const uint32_t order, const double p) const override;

    virtual double ParamLength() const;

    /**
     * @brief Curve1d to string
     *
     * @return string
     */
    virtual std::string String() const;

protected:
    void ComputeCoefficients(const double x0, const double dx0, const double ddx0, const double x1, const double dx1,
                             const double ddx1, const double param);

protected:
    double param_ = 0.0;

    // f = sum(coef_[i] * x^i), i from 0 to 5
    std::array<double, 6> coef_{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
    std::array<double, 3> start_condition_{{0.0, 0.0, 0.0}};
    std::array<double, 3> end_condition_{{0.0, 0.0, 0.0}};
};

}  // namespace math

#endif
