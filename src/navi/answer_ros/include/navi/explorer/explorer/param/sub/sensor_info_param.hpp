/*
 * @file	: sensor_info_param.hpp
 * @date	: Jun 12, 2025
 * @author	:"HaeChang, Kim(henry)" (henry@navifra.com)"
 * @brief	: edge contour detection from image
 * @remark	:
 * @warning	:
 * Copyright(C) 2025 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef EXPLORER_SENSOR_INFO_PARAM_HPP_
#define EXPLORER_SENSOR_INFO_PARAM_HPP_

namespace NVFR {

struct SensorInfoParam_t
{
public:
  // sensor info
  float f_max_ray_range_m = 10.0f;
  float f_start_angle_deg = 0.0f;
  float f_angle_range_deg = 360.0f;
  float f_angle_res_deg   = 1.0f;

};

} // namespace NVFR

#endif
