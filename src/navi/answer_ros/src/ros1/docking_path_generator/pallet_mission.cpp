#include "ros1/docking_path_generator/pallet_mission.hpp"

#include "logger/logger.h"

namespace NVFR {

PalletMission::PalletMission(
    bool b_local,
    const Pose& o_robot2sensor,
    const Pose& o_map2object,
    const Pose& o_object2goal,
    const Pose& o_fNode,
    const DPG_Param_t& st_dpg_param)
    : VirtualMission(b_local, o_robot2sensor, o_map2object, o_object2goal, o_fNode, st_dpg_param)
{
    o_docking_ptr_ = std::make_shared<Docking>(o_robot2sensor, o_map2object, o_object2goal, st_dpg_param.st_pallet_docking_param);

    o_docking_ptr_->TurnOn();
    o_loop_thread_.Start();

    LOG_INFO("PalletMission is initialized");
}

}  // namespace NVFR
