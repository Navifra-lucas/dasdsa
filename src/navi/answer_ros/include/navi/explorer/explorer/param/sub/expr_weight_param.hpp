/*
 * @file	: expr_weight_param.hpp
 * @date	: Jun 12, 2025
 * @author	:"HaeChang, Kim(henry)" (henry@navifra.com)"
 * @brief	: edge contour detection from image
 * @remark	:
 * @warning	:
 * Copyright(C) 2025 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef EXPLORER_EXPR_WEIGHT_PARAM_HPP_
#define EXPLORER_EXPR_WEIGHT_PARAM_HPP_

namespace NVFR {

struct ExprWeightParam_t
{
public:
  // weight of explore cost
  float f_distance = 1.0f;
  float f_unknown_gain = 1.0f;

};

} // namespace NVFR

#endif
