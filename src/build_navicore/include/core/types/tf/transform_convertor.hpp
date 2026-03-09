/*
 * @file	: TransformConvertor.hpp
 * @date	: Jan. 1, 2022
 * @author	: "Joongtae, Park(brain)" (founder@navifra.com)"
 * @brief	: coordinate transform수행
 * @remark	:
 * @warning	:
 * 	Copyright(C) 2022 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef NAVIFRA_TRANSFORMER_HPP_
#define NAVIFRA_TRANSFORMER_HPP_

#include "core_calculator/core_calculator.hpp"
#include "msg_information/sensor_information/scan.hpp"
#include "pos/pos.hpp"
#include "tf/transform_convertor.hpp"

#include <iostream>
#include <vector>
namespace NaviFra {
class TransformConvertor {
public:
    TransformConvertor(){};
    virtual ~TransformConvertor(){};

    static Pos ConvertToGlobalPosition_(const Pos& o_local_pos, const Pos& o_origin_pos);
    static Pos ConvertToLocalPosition_(const Pos& o_origin_pos, const Pos& o_global_pos);
    static std::vector<Pos> ConvertToGlobalPositionVector_(const std::vector<Pos>& vec_local_pos, const Pos& o_origin_pos);
    static std::vector<Pos> ConvertToLocalPositionVector_(const std::vector<Pos>& vec_global_pos, const Pos& o_origin_pos);
    static std::vector<SimplePos> ConvertScanToPosition_(
        const Scan_t& st_laser_scan, float f_scan_angle_inc = 1.f, float f_min_dist = 0.05);
};

}  // namespace NaviFra

#endif
