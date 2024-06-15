#ifndef __MATH_UTIL_H__
#define __MATH_UTIL_H__

#include <cmath>
#include <iostream>

#include "nav2_regulated_pure_pursuit_controller/common/curve_point.h"

using namespace common;
namespace math
{
class MathUtil
{
public:
    MathUtil()  = default;
    ~MathUtil() = default;

    /**
     * @brief      normalize angle to [-PI, PI).
     *
     * @param[in]  angle  The original value of the angle.
     *
     * @return     The normalized value of the angle.
     */
    static double NormalizeAngle(const double angle);

    /**
     * @brief      Spherical linear interpolation between two angles. The two angles
     *             are within range [-M_PI, M_PI).
     *
     * @param[in]  a0    The value of the first angle.
     * @param[in]  t0    The interpolation parameter of the first angle.
     * @param[in]  a1    The value of the second angle.
     * @param[in]  t1    The interpolation parameter of the second angle.
     * @param[in]  t     The interpolation parameter for interpolation.
     *
     * @return     Interpolated angle.
     */
    static double Slerp(const double a0, const double t0, const double a1, const double t1, const double t);

    static CurvePoint InterpolateUsingLinearApproximation(const CurvePoint& p0, const CurvePoint& p1, const double s);

    static double LinearInterpolation(const double x1, const double y1, const double x2, const double y2,
                                      const double x);

    /**
     * @brief a classic function, it's another name is Logistic function.
     *
     * @details The sigmoid function is also called the logistic function, which is used for hidden layer neuron output.
     * The value range is (0,1). It can map a real number to the range of (0,1) and can be used for binary
     * classification. The effect is better when the feature difference is more complicated or the difference is not
     * particularly large.
     */
    static inline double Sigmoid(double x)
    {
        return 1.0 / (1.0 + std::exp(-x));
    }
};
}  // namespace math

#endif
