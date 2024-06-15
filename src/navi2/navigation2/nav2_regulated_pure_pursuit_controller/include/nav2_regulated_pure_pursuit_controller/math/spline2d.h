/*************************************************************************
    > File Name: csv_reader.h
    > Author: TAI Lei
    > Mail: ltai@ust.hk
    > Created Time: Wed Mar 27 12:49:12 2019
 ************************************************************************/

#ifndef _CUBIC_SPLINE_H
#define _CUBIC_SPLINE_H

#include <Eigen/Eigen>
#include <array>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace cublic_spline
{
// using std::vector<float>   = std::vector<float>;
// using std::array<float, 2> = std::array<float, 2>;
// using Vec_Poi              = std::vector<std::array<float, 2>>;

class Spline
{
public:
    std::vector<float> x;
    std::vector<float> y;
    int                nx;
    std::vector<float> h;
    std::vector<float> a;
    std::vector<float> b;
    std::vector<float> c;
    std::vector<float> d;

    Spline(){};
    // d_i * (x-x_i)^3 + c_i * (x-x_i)^2 + b_i * (x-x_i) + a_i
    Spline(std::vector<float> x_, std::vector<float> y_);

    Spline(std::vector<float> x_, std::vector<float> y_, float start_slope, float end_slope);

    float calc(float t);

    float calc_d(float t);

    float calc_dd(float t);

private:
    Eigen::MatrixXf calc_A();
    Eigen::VectorXf calc_B();

    Eigen::MatrixXf calc_A_();
    Eigen::VectorXf calc_B_(float start_slope, float end_slope);

    int bisect(float t, int start, int end);
};

class Spline2D
{
public:
    Spline             sx;
    Spline             sy;
    std::vector<float> s;

    Spline2D(std::vector<float> x, std::vector<float> y);

    Spline2D(std::vector<float> x, std::vector<float> y, float start_angle, float end_angle);  // angle:radian

    std::array<float, 2> calc_postion(float s_t);

    float calc_curvature(float s_t);

    float calc_yaw(float s_t);

private:
    std::vector<float> calc_s(std::vector<float> x, std::vector<float> y);
};

}  // namespace cublic_spline
#endif
