#ifndef POLYNOMINAL_PATH_HPP
#define POLYNOMINAL_PATH_HPP

#include "core/util/logger.hpp"

#include <cmath>
#include <eigen3/Eigen/Dense>
#include <vector>

namespace NaviFra {

class PolynominalPath {
public:
    PolynominalPath(float p0, float p1, float p2, float p3, float p4, float p5)
    {
        p0_ = p0;
        p1_ = p1;
        p2_ = p2;
        p3_ = p3;
        p4_ = p4;
        p5_ = p5;
    };
    virtual ~PolynominalPath(){};

    float GetPos(float t);

    float GetVel(float t);

    float GetAcc(float t);

    std::vector<float> GetPosVector(const std::vector<float>& t_array);

    std::vector<float> GetVelVector(const std::vector<float>& t_array);

    std::vector<float> GetAccArray(const std::vector<float>& t_array);

    float p0_ = 0;
    float p1_ = 0;
    float p2_ = 0;
    float p3_ = 0;
    float p4_ = 0;
    float p5_ = 0;
};

float CalCurv(float vx, float vy, float ax, float ay);

void RecalAngVelAccArray(
    std::vector<float>& vx_array, std::vector<float>& vy_array, std::vector<float>& ax_array, std::vector<float>& ay_array,
    std::vector<float>& ang_array, std::vector<float>& vel_array);

std::vector<float> CalCurvArray(
    const std::vector<float>& vx_array, const std::vector<float>& vy_array, const std::vector<float>& ax_array,
    const std::vector<float>& ay_array);

std::vector<float> CalCurvArray2(
    const std::vector<float>& px_array, const std::vector<float>& py_array, const std::vector<float>& ang_array);

float PathDistanceOptimization(
    float x0, float y0, float a0, float c0, float v0, float x1, float y1, float a1, float c1, float v1, float f_ds);

}  // namespace NaviFra
#endif