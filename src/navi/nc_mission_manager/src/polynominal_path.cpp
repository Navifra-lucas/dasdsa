#include "polynominal_path.hpp"

namespace NaviFra {
    float PolynominalPath::GetPos(float t)
    {
        return p0_ * t * t * t * t * t + p1_ * t * t * t * t + p2_ * t * t * t + p3_ * t * t + p4_ * t + p5_;
    }
    float PolynominalPath::GetVel(float t)
    {
        return 5 * p0_ * t * t * t * t + 4 * p1_ * t * t * t + 3 * p2_ * t * t + 2 * p3_ * t + p4_;
    }
    float PolynominalPath::GetAcc(float t)
    {
        return 20 * p0_ * t * t * t + 12 * p1_ * t * t + 6 * p2_ * t + 2 * p3_;
    }
    std::vector<float> PolynominalPath::GetPosVector(const std::vector<float>& t_array)
    {
        std::vector<float> p_array;
        for (float t : t_array) {
            p_array.emplace_back(GetPos(t));
        }
        return p_array;
    }
    std::vector<float> PolynominalPath::GetVelVector(const std::vector<float>& t_array)
    {
        std::vector<float> v_array;
        for (float t : t_array) {
            v_array.emplace_back(GetVel(t));
        }
        return v_array;
    }
    std::vector<float> PolynominalPath::GetAccArray(const std::vector<float>& t_array)
    {
        std::vector<float> a_array;
        for (float t : t_array) {
            a_array.emplace_back(GetAcc(t));
        }
        return a_array;
    }

    float GetNextTime(float v, float t)
    {
        float f_next_t = t + 0.01/v;
        return f_next_t;
    }

    float CalCurv(float vx, float vy, float ax, float ay)
    {
        return (vx * ay - vy * ax) / pow(sqrt(vx * vx + vy * vy), 3);
    }

    
    void RecalAngVelAccArray(std::vector<float>& vx_array, std::vector<float>& vy_array, 
                                std::vector<float>& ax_array, std::vector<float>& ay_array, 
                                std::vector<float>& ang_array, std::vector<float>& vel_array)
    {
        size_t len = vx_array.size();
        if (len != vy_array.size() || len != ax_array.size() || len != ay_array.size()) {
            LOG_ERROR("[Error] recal_ang_vel_acc_array : size of array is not equal");
            return;
        }
        ang_array.resize(len);
        vel_array.resize(len);
        for (size_t i = 0; i < len; ++i) {
            vel_array.at(i) = sqrt(vx_array.at(i) * vx_array.at(i) + vy_array.at(i) * vy_array.at(i));
            float angle = atan2(vy_array.at(i), vx_array.at(i));
            ang_array.at(i) = angle;
            float ax = cos(angle) * ax_array.at(i) + sin(angle) * ay_array.at(i);
            float ay = -sin(angle) * ax_array.at(i) + cos(angle) * ay_array.at(i);
            ax_array.at(i) = ax;
            ay_array.at(i) = ay;
        }
    }
    
    std::vector<float> CalCurvArray(const std::vector<float>& vx_array, const std::vector<float>& vy_array, 
                                const std::vector<float>& ax_array, const std::vector<float>& ay_array)
    {
        size_t len = vx_array.size();
        if (len != vy_array.size() || len != ax_array.size() || len != ay_array.size()) {
            LOG_ERROR("[Error] Noah_check cal_curv_array : size of array is not equal");
            return std::vector<float>();
        }
        std::vector<float> c_array(len);
        for (size_t i = 0; i < len; ++i) {
            c_array.at(i) = CalCurv(vx_array.at(i), vy_array.at(i), ax_array.at(i), ay_array.at(i));
        }
        return c_array;
    }
    std::vector<float> CalCurvArray2(const std::vector<float>& px_array, const std::vector<float>& py_array, 
                                const std::vector<float>& ang_array)
    {
        size_t len = ang_array.size();
        if (len != px_array.size() || len != py_array.size()) {
            LOG_ERROR("[Error] Noah_check cal_curv_array2 : size of array is not equal");
            return std::vector<float>();
        }
        std::vector<float> c_array(len);
        for (size_t i = 0; i < len - 1; ++i) {
            float ds = sqrt(pow(px_array.at(i + 1) - px_array.at(i), 2) + pow(py_array.at(i + 1) - py_array.at(i), 2));
            float da = ang_array.at(i + 1) - ang_array.at(i);
            c_array.at(i) = da / ds;
        }
        c_array.at(len - 1) = c_array.at(len - 2);
        return c_array;
    }

    float PathDistanceOptimization(float x0, float y0, float a0, float c0, float v0, float x1, float y1, float a1, float c1, float v1, float f_ds)
    {
        float V_min = std::min(v0, v1);
        float V_max = std::max(v0, v1);
        float T = 2 * f_ds / (V_min + V_max);

        // Polynomial function
        float T_5 = pow(T, 5);
        float T_4 = pow(T, 4);
        float T_3 = pow(T, 3);
        float T_2 = pow(T, 2);

        Eigen::Matrix3d mat_A;
        mat_A << T_5, T_4, T_3, 
                5 * T_4, 4 * T_3, 3 * T_2, 
                20 * T_3, 12 * T_2, 6 * T;

        if (mat_A.determinant() == 0) {
            LOG_INFO("[Error] (T= %f ) Det(A) == 0", T);
            return 0;
        }

        Eigen::Matrix3d inv_A = mat_A.inverse();
        
        // x(t) coefficients
        Eigen::Vector3d vec_b;
        vec_b << x1 + 0.5*c0*v0*v0*sin(a0)*T_2 - v0*cos(a0)*T - x0,
                v1*cos(a1) + c0*v0*v0*sin(a0)*T - v0*cos(a0),
                c0*v0*v0*sin(a0) - c1*v1*v1*sin(a1);

        Eigen::Vector3d vec_p = inv_A * vec_b;
        PolynominalPath x_poly_tmp(vec_p(0), vec_p(1), vec_p(2), -0.5*c0*v0*v0*sin(a0), v0*cos(a0), x0);

        // y(t) coefficients
        vec_b << y1 - 0.5*c0*v0*v0*cos(a0)*T_2 - v0*sin(a0)*T - y0,
                v1*sin(a1) - c0*v0*v0*cos(a0)*T - v0*sin(a0),
                c1*v1*v1*cos(a1) - c0*v0*v0*cos(a0);

        vec_p = inv_A * vec_b;

        PolynominalPath y_poly_tmp(vec_p(0), vec_p(1), vec_p(2), 0.5*c0*v0*v0*cos(a0), v0*sin(a0), y0);
        std::vector<float> t_array;
        for (int i = 0; i <= 10; ++i) {
            t_array.emplace_back(i * (T/10));
        }

        std::vector<float> x_array = x_poly_tmp.GetPosVector(t_array);
        std::vector<float> y_array = y_poly_tmp.GetPosVector(t_array);

        float f_distance = 0;
        for (int i = 1 ; i < (int)x_array.size() ; i++)
        {
            f_distance += (float)hypot(x_array.at(i) - x_array.at(i-1), y_array.at(i) - y_array.at(i-1));
        }

        return f_distance;
    }
}