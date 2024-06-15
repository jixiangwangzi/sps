#include "nav2_regulated_pure_pursuit_controller/math/curve_1d.h"

namespace math
{
std::ostream& operator<<(std::ostream& os, const math::Curve1d& curve)
{
    os << curve.String();
    return os;
}

std::ostream& operator<<(std::ostream& os, const std::shared_ptr<math::Curve1d>& curve)
{
    os << curve->String();
    return os;
}

}  // namespace math
