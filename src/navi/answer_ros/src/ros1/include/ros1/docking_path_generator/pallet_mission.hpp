#ifndef PALLET_MISSION_HPP_
#define PALLET_MISSION_HPP_

#include <mutex>

#include "virtual_mission.hpp"

namespace NVFR {

class PalletMission : public VirtualMission
{
public:
    PalletMission(
        bool b_local,
        const Pose& o_robot2sensor,
        const Pose& o_map2object,
        const Pose& o_object2goal,
        const Pose& o_fNode,
        const DPG_Param_t& st_dpg_param);
    virtual ~PalletMission() = default;

private:

};

}  // namespace NVFR

#endif
