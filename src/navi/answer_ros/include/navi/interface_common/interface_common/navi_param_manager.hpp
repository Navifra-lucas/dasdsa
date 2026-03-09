/*
 * @file	: navi_param_manager.hpp
 * @date	: Mar 27, 2025
 * @author	:"HaeChang, Kim(henry)" (henry@navifra.com)"
 * @brief	: parameter manager
 * @remark	:
 * @warning	:
 * Copyright(C) 2025 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef NAVI_PARAM_MANAGER_HPP_
#define NAVI_PARAM_MANAGER_HPP_

#include <string>

#include "common/configurator.h"

#include "utils/param/navigator_param.hpp"

namespace NVFR {

namespace NaviParamManager {

NavigatorParam_t LoadNaviParam();

} // namespace NaviParamManager

} // namespace NVFR

#endif
