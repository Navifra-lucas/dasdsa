#ifndef DPG_PARAM_MANAGER_HPP_
#define DPG_PARAM_MANAGER_HPP_

#include <string>

#include "common/configurator.h"

#include "ros1/docking_path_generator/dpg_param.hpp"

namespace NVFR {

namespace DPGParamManager {

DPG_Param_t LoadDPGParam();

}

}  // namespace NVFR

#endif
