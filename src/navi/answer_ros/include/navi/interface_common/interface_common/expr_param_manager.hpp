/*
 * @file	: expr_param_manager.hpp
 * @date	: Mar 27, 2025
 * @author	:"HaeChang, Kim(henry)" (henry@navifra.com)"
 * @brief	: parameter manager
 * @remark	:
 * @warning	:
 * Copyright(C) 2025 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef EXPR_PARAM_MANAGER_HPP_
#define EXPR_PARAM_MANAGER_HPP_

#include <string>

#include "common/configurator.h"

#include "explorer/param/expr_param.hpp"

namespace NVFR {

namespace ExprParamManager {

ExprParam_t LoadExprParam();

} // namespace ExprParamManager

} // namespace NVFR

#endif
