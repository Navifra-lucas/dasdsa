#ifndef WINGBODY_MISSION_HPP_
#define WINGBODY_MISSION_HPP_

#include "virtual_mission.hpp"

namespace NVFR {

class WingBodyMission : public VirtualMission
{
public:
    WingBodyMission(
        bool b_local,
        const Pose& o_robot2sensor,
        const Pose& o_map2object,
        const Pose& o_object2goal,
        const Pose& o_fNode,
        const DPG_Param_t& st_dpg_param);
    virtual ~WingBodyMission() = default;

private:

};

}  // namespace NVFR

#endif
