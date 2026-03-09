#ifndef DOCKING_INFO_HPP_
#define DOCKING_INFO_HPP_

#include <iostream>
#include <string>

#include "utils/pose.hpp"

namespace NVFR {

struct DockingInfo
{
    /**
    * @note type: int
    * @param NONE, PALLET, WINGBODY, GOBACK
    */
    enum CMD : int
    {
        NONE = 0,
        PALLET = 1,
        WINGBODY,
        GOBACK,
    };
    CMD n_mission_type = CMD::NONE;

    Pose o_fNode;
    Pose o_object2goal;
    Pose o_pose;
    bool b_local = false;

    bool ParseJson(const std::string& s_data);
};

}  // namespace NVFR

#endif
