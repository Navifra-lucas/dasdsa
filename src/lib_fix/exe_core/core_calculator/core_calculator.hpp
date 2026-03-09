#ifndef CORE_CALCULATOR_HPP_
#define CORE_CALCULATOR_HPP_

#include "pos/pos.hpp"
#include "simplepos/simplepos.hpp"
#include "util/ansi_color.h"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <random>
#include <type_traits>
#include <vector>

#define RADtoDEG (57.29577951f)  ///< 180.0/PI
#define DEGtoRAD (0.017453292f)  ///< PI/180.0
using namespace std;
namespace NaviFra {
class CoreCalculator {
    CoreCalculator(){};
    virtual ~CoreCalculator(){};

public:
    static inline int ToSign_(float value) { return (value > 0) - (value < 0); }

    static float CalcPosDistance_(const Pos& o_pos1);
    static float CalcPosDistance_(const Pos& o_pos1, const Pos& o_pos2);

    static int FindMinDistanceIdxFromPosVector_(const Pos& o_pos, const std::vector<Pos>& vec_data);
    static int FindMinDistanceIdxFromPosVectorFarPoint_(const Pos& o_pos, const std::vector<Pos>& vec_data);
    static int FindIndexAtDistance_(
        const std::vector<Pos>& vec_path_pos, int n_start_idx, float f_apart_length, const bool b_is_reverse = false);

    static float CalcDistanceFromDotToLine_(const Pos& o_line_s, const Pos& o_line_e, const Pos& o_target_pos);
    static float CalcPosDotProduct_(const Pos& A, const Pos& B);
    static float WrapAnglePiToPiRad_(float angle_rad);
    static float WrapAnglePiToPiDeg_(float angle_deg);
    static float CalcVectorPosDistance_(const std::vector<Pos>& vec_pos);

    static Pos CalcUnitVector_(const float& f_dist_m, const float& f_radian);

    static Pos CalcPosDotOnLine_(const Pos& o_line_s, const Pos& o_line_e, const Pos& o_target_pos);
    static Pos TransformRotationRad_(const Pos& o_pos, float f_rad);
    static Pos TransformRotationDeg_(const Pos& o_pos, float f_deg);
    static Pos TransformPos_(const Pos& target, const Pos& relation);

    static std::vector<Pos> GetRayPosToTarget_(const Pos& o_start_pos, const Pos& o_target_pos, float f_dist_m = 0.1);
    static float CalcAngleDomainRad_(float angle_rad);
    static float CalcAngleDomainDeg_(float angle_deg);
    static int sign(float val);

    /**
     * @brief check equal values, a == b ? (luna)
     * @param a
     * @param b
     * @return true if equal, else not
     */
    static constexpr bool isEqual(float a, float b) { return !isgreater(a, b) && !isless(a, b); }

    /**
     * @brief get '-, +' (luna)
     * @return return 1 if positive, else if -1 negative, zero returns 0
     */
    static const int Sign(float a) { return isgreater(a, 0.0f) ? 1 : isless(a, 0.0f) ? -1 : 0; }

    /**
     * @brief compare values val > comp (luna)
     * @param val input value
     * @param comp comparing standard value
     * @return true if big, else false
     */
    static constexpr bool isBig(float val, float comp) { return std::isgreater(val, comp); }

    /**
     * @brief compare values val < comp (luna)
     * @param val input value
     * @param comp comparing standard value
     * @return true if small, else false
     */
    static constexpr bool isSmall(float val, float comp) { return std::isless(val, comp); }

    /**
     * @brief get radian domain (luna)
     * @param rad input radian domain [rad]
     * @return radian domain (-3.14 ~ 3.14)
     */
    static const float RadDomain(float rad)
    {
        float result = fmod(rad, 2.0f * M_PI);
        return std::isgreater(result, M_PI) ? result - 2.f * M_PI : std::isless(result, -M_PI) ? result + 2.f * M_PI : result;
    }

    /**
     * @brief get degree domain (luna)
     * @param deg input degree domain [rad]
     * @return degree domain (-180 ~ 180)
     */
    static float DegDomain(float deg)
    {
        float result = fmod(deg, 360.f);
        return std::isgreater(result, 180.f) ? result - 360.f : std::isless(result, -180.f) ? result + 360.f : result;
    }

    /**
     * @brief constraing calculation: rw = v
     * @param f_path_radius_m 	[m]
     * @param f_w_deg_s 		[deg/s]
     * @return v[m/s] constraint
     */
    static float CalcConstraintVfromDegW_(float f_path_radius, float f_w)
    {
        return CalcConstraintVfromRadiusWrad_(f_path_radius, f_w * DEGtoRAD);
    }

    /**
     * @brief constraing calculation: v/r = w
     * @param f_path_radius_m 	[m]
     * @param f_v_m_s 			[m/s]
     * @return w[rad/s] constraint
     */
    static float CalcConstraintWradfromRadiusV_(float f_path_radius, float f_v) { return (f_v / f_path_radius); }

    /**
     * @brief constraing calculation: rw = v
     * @param f_path_radius_m 	[m]
     * @param f_w_rad_s 		[rad/s]
     * @return v[m/s] constraint
     */
    static float CalcConstraintVfromRadiusWrad_(float f_path_radius, float f_w) { return (f_path_radius * f_w); }

    /**
     * @brief constraing calculation: v/w = r
     * @param f_v [m/s]
     * @param f_w [rad/s]
     * @return r[m] constraint
     */
    static float CalcConstraintRadiusfromVW_(float f_v, float f_w) { return (f_v / f_w); }

    /**
     * @brief set boundary limit
     * @param f_val reference
     * @param f_min minimum limit
     * @param f_max maximum limit
     */
    static void BoundValue(float& f_val, const float f_min, const float f_max) { f_val = std::max(std::min(f_max, f_val), f_min); }

    /**
     * @brief check minimum value in absolute
     * @param f_val minimum value
     * @param f_check value to check
     */
    static void BoundMinValue(float& f_check, const float& f_val)
    {
        f_check = (isSmall(fabs(f_check), f_val)) ? Sign(f_check) * f_val : f_check;
    }
};
}  // namespace NaviFra
#endif
