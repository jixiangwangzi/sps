#ifndef __POINT_H__
#define __POINT_H__

namespace common
{

class Point
{
public:
    Point()  = default;
    ~Point() = default;

    void  SetX(float x) noexcept;
    float GetX() const noexcept;

    void  SetY(float y) noexcept;
    float GetY() const noexcept;

    void  SetZ(float z) noexcept;
    float GetZ() const noexcept;

protected:
    float x_ = 0.0;
    float y_ = 0.0;
    float z_ = 0.0;
};
}  // namespace common

#endif
