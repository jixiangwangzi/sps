#ifndef _HOLO_PLANNING_MATH_CURVE_1D_H_
#define _HOLO_PLANNING_MATH_CURVE_1D_H_

#include <iostream>
#include <memory>

namespace math
{
/**
 * @brief      Curve1d class
 *
 *             Base type for various types of 1-dimensional curves
 */
class Curve1d
{
public:
    Curve1d() = default;

    virtual ~Curve1d() = default;

    virtual double Evaluate(const uint32_t order, const double param) const = 0;
    virtual double ParamLength() const                                      = 0;

    /**
     * @brief Curve1d to string
     *
     * @return string
     */
    virtual std::string String() const = 0;
};

std::ostream& operator<<(std::ostream& os, const math::Curve1d& curve);
std::ostream& operator<<(std::ostream& os, const std::shared_ptr<math::Curve1d>& curve);

}  // namespace math

#endif
