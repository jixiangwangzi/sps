#include "nav2_regulated_pure_pursuit_controller/math/spline2d.h"

#include <cmath>

namespace cublic_spline
{
std::vector<float> vec_diff(std::vector<float> input)
{
    std::vector<float> output;
    for (unsigned int i = 1; i < input.size(); i++)
    {
        output.push_back(input[i] - input[i - 1]);
    }

    return output;
}

std::vector<float> cum_sum(std::vector<float> input)
{
    std::vector<float> output;
    float              temp = 0;
    for (unsigned int i = 0; i < input.size(); i++)
    {
        temp += input[i];
        output.push_back(temp);
    }
    return output;
}

// d_i * (x-x_i)^3 + c_i * (x-x_i)^2 + b_i * (x-x_i) + a_i
Spline::Spline(std::vector<float> x_, std::vector<float> y_) : x(x_), y(y_), nx(x_.size()), h(vec_diff(x_)), a(y_)
{
    Eigen::MatrixXf A         = calc_A();
    Eigen::VectorXf B         = calc_B();
    Eigen::VectorXf c_eigen   = A.colPivHouseholderQr().solve(B);
    float*          c_pointer = c_eigen.data();

    c.assign(c_pointer, c_pointer + c_eigen.rows());

    for (int i = 0; i < nx - 1; i++)
    {
        d.push_back((c[i + 1] - c[i]) / (3.0 * h[i]));
        b.push_back((a[i + 1] - a[i]) / h[i] - h[i] * (c[i + 1] + 2 * c[i]) / 3.0);
    }
}

// d_i * (x-x_i)^3 + c_i * (x-x_i)^2 + b_i * (x-x_i) + a_i
Spline::Spline(std::vector<float> x_, std::vector<float> y_, float start_slope, float end_slope)
  : x(x_), y(y_), nx(x_.size()), h(vec_diff(x_)), a(y_)
{
    Eigen::MatrixXf A         = calc_A_();
    Eigen::VectorXf B         = calc_B_(start_slope, end_slope);
    Eigen::VectorXf c_eigen   = A.colPivHouseholderQr().solve(B);
    float*          c_pointer = c_eigen.data();

    c.assign(c_pointer, c_pointer + c_eigen.rows());

    for (int i = 0; i < nx - 1; i++)
    {
        d.push_back((c[i + 1] - c[i]) / (3.0 * h[i]));
        b.push_back((a[i + 1] - a[i]) / h[i] - h[i] * (c[i + 1] + 2 * c[i]) / 3.0);
    }
}

float Spline::calc(float t)
{
    if (t < x.front() || t > x.back())
    {
        throw std::invalid_argument("received value out of the pre-defined range");
    }

    int   seg_id = bisect(t, 0, nx);
    float dx     = t - x[seg_id];
    return a[seg_id] + b[seg_id] * dx + c[seg_id] * dx * dx + d[seg_id] * dx * dx * dx;
}

float Spline::calc_d(float t)
{
    if (t < x.front() || t > x.back())
    {
        throw std::invalid_argument("received value out of the pre-defined range");
    }

    int   seg_id = bisect(t, 0, nx - 1);
    float dx     = t - x[seg_id];
    return b[seg_id] + 2 * c[seg_id] * dx + 3 * d[seg_id] * dx * dx;
}

float Spline::calc_dd(float t)
{
    if (t < x.front() || t > x.back())
    {
        throw std::invalid_argument("received value out of the pre-defined range");
    }

    int   seg_id = bisect(t, 0, nx);
    float dx     = t - x[seg_id];
    return 2 * c[seg_id] + 6 * d[seg_id] * dx;
}

Eigen::MatrixXf Spline::calc_A()
{
    Eigen::MatrixXf A = Eigen::MatrixXf::Zero(nx, nx);
    A(0, 0)           = 1;
    for (int i = 0; i < nx - 1; i++)
    {
        if (i != nx - 2)
        {
            A(i + 1, i + 1) = 2 * (h[i] + h[i + 1]);
        }
        A(i + 1, i) = h[i];
        A(i, i + 1) = h[i];
    }
    A(0, 1)           = 0.0;
    A(nx - 1, nx - 2) = 0.0;
    A(nx - 1, nx - 1) = 1.0;

    return A;
}

Eigen::VectorXf Spline::calc_B()
{
    Eigen::VectorXf B = Eigen::VectorXf::Zero(nx);
    for (int i = 0; i < nx - 2; i++)
    {
        B(i + 1) = 3.0 * (a[i + 2] - a[i + 1]) / h[i + 1] - 3.0 * (a[i + 1] - a[i]) / h[i];
    }

    return B;
}

Eigen::MatrixXf Spline::calc_A_()
{
    Eigen::MatrixXf A = Eigen::MatrixXf::Zero(nx, nx);
    A(0, 0)           = 2 * h[0];
    for (int i = 0; i < nx - 1; i++)
    {
        if (i != nx - 2)
        {
            A(i + 1, i + 1) = 2 * (h[i] + h[i + 1]);
        }
        A(i + 1, i) = h[i];
        A(i, i + 1) = h[i];
    }
    A(0, 1)           = h[0];
    A(nx - 1, nx - 2) = h[nx - 2];
    A(nx - 1, nx - 1) = 2 * h[nx - 2];

    return A;
}

Eigen::VectorXf Spline::calc_B_(float start_slope, float end_slope)
{
    Eigen::VectorXf B = Eigen::VectorXf::Zero(nx);
    for (int i = 0; i < nx - 2; i++)
    {
        B(i + 1) = 3.0 * (a[i + 2] - a[i + 1]) / h[i + 1] - 3.0 * (a[i + 1] - a[i]) / h[i];
    }

    B(0)      = 3.0 * ((a[1] - a[0]) / h[0] - start_slope);
    B(nx - 1) = 3.0 * (end_slope - (a[nx - 1] - a[nx - 2]) / h[nx - 2]);

    return B;
}

int Spline::bisect(float t, int start, int end)
{
    int mid = (start + end) / 2;
    if (t == x[mid] || end - start <= 1)
    {
        return mid;
    }
    else if (t > x[mid])
    {
        return bisect(t, mid, end);
    }
    else
    {
        return bisect(t, start, mid);
    }
}

Spline2D::Spline2D(std::vector<float> x, std::vector<float> y)
{
    s = calc_s(x, y);

    sx = Spline(s, x);
    sy = Spline(s, y);
}

Spline2D::Spline2D(std::vector<float> x, std::vector<float> y, float start_angle, float end_angle)
{
    s = calc_s(x, y);

    sx = Spline(s, x, std::cos(start_angle), std::cos(end_angle));
    sy = Spline(s, y, std::sin(start_angle), std::sin(end_angle));
}

std::array<float, 2> Spline2D::calc_postion(float s_t)
{
    float x = sx.calc(s_t);
    float y = sy.calc(s_t);
    return {{x, y}};
}

float Spline2D::calc_curvature(float s_t)
{
    float dx  = sx.calc_d(s_t);
    float ddx = sx.calc_dd(s_t);
    float dy  = sy.calc_d(s_t);
    float ddy = sy.calc_dd(s_t);
    return (ddy * dx - ddx * dy) / std::pow((dx * dx + dy * dy), 1.5);
}

float Spline2D::calc_yaw(float s_t)
{
    float dx = sx.calc_d(s_t);
    float dy = sy.calc_d(s_t);
    return std::atan2(dy, dx);
}

std::vector<float> Spline2D::calc_s(std::vector<float> x, std::vector<float> y)
{
    std::vector<float> ds;
    std::vector<float> out_s{0};
    std::vector<float> dx = vec_diff(x);
    std::vector<float> dy = vec_diff(y);

    for (unsigned int i = 0; i < dx.size(); i++)
    {
        ds.push_back(std::sqrt(dx[i] * dx[i] + dy[i] * dy[i]));
    }

    std::vector<float> cum_ds = cum_sum(ds);
    out_s.insert(out_s.end(), cum_ds.begin(), cum_ds.end());
    return out_s;
}

}  // namespace cublic_spline
