#ifndef DPG_PARAM_HPP_
#define DPG_PARAM_HPP_

#include "utils/param/sub/docking_param.hpp"

namespace NVFR {

struct DPG_Param_t
{
    int n_replan_period_ms = 500;

    double d_far_from_path_dist_m = 0.2;
    double d_goback_on_path_dist_m = 1;  // GoBack 경로 이탈 허용 거리 (m)
    double d_straight_enter_dist_m = 2.0;

    DockingParam_t st_pallet_docking_param;
    DockingParam_t st_wingbody_docking_param;

};

}  // namespace NVFR

#endif
