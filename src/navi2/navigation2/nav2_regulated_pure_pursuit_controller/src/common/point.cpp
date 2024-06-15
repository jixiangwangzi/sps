#include "nav2_regulated_pure_pursuit_controller/common/point.h"

namespace common
{
void Point::SetX(float x) noexcept
{
    x_ = x;
}

float Point::GetX() const noexcept
{
    return x_;
}

void Point::SetY(float y) noexcept
{
    y_ = y;
}

float Point::GetY() const noexcept
{
    return y_;
}

void Point::SetZ(float z) noexcept
{
    z_ = z;
}

float Point::GetZ() const noexcept
{
    return z_;
}
}  // namespace common
