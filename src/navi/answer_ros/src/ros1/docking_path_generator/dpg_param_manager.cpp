#include "ros1/docking_path_generator/dpg_param_manager.hpp"

#include "ros/package.h"

namespace NVFR {

namespace DPGParamManager {

DPG_Param_t LoadDPGParam()
{
    ANSWER::Configurator::GetInstance().LoadParameters("dpg");
    ANSWER::Configurator::GetInstance().LoadParameters("pallet_docking");
    ANSWER::Configurator::GetInstance().LoadParameters("wingbody_docking");

    DPG_Param_t st_param;

    // dpg
    st_param.n_replan_period_ms = ANSWER::Configurator::GetInstance()
        .GetParamValue("dpg", "n_replan_period_ms")
        .convert<int>();
    st_param.d_far_from_path_dist_m = ANSWER::Configurator::GetInstance()
        .GetParamValue("dpg", "d_far_from_path_dist_m")
        .convert<double>();
    st_param.d_goback_on_path_dist_m = ANSWER::Configurator::GetInstance()
        .GetParamValue("dpg", "d_goback_on_path_dist_m")
        .convert<double>();
    st_param.d_straight_enter_dist_m = ANSWER::Configurator::GetInstance()
        .GetParamValue("dpg", "d_straight_enter_dist_m")
        .convert<double>();

    // pallet docking
    st_param.st_pallet_docking_param.n_delay_msec = ANSWER::Configurator::GetInstance()
        .GetParamValue("pallet_docking", "n_delay_msec")
        .convert<int>();
    st_param.st_pallet_docking_param.d_robot2sensor_x_m = ANSWER::Configurator::GetInstance()
        .GetParamValue("pallet_docking", "d_robot2sensor_x_m")
        .convert<double>();
    st_param.st_pallet_docking_param.d_robot2sensor_y_m = ANSWER::Configurator::GetInstance()
        .GetParamValue("pallet_docking", "d_robot2sensor_y_m")
        .convert<double>();
    st_param.st_pallet_docking_param.d_robot2sensor_deg = ANSWER::Configurator::GetInstance()
        .GetParamValue("pallet_docking", "d_robot2sensor_deg")
        .convert<double>();
    st_param.st_pallet_docking_param.st_dekf_param.n_max_buffer_size = ANSWER::Configurator::GetInstance()
        .GetParamValue("pallet_docking", "st_dekf_param", "n_max_buffer_size")
        .convert<int>();
    st_param.st_pallet_docking_param.st_dekf_param.d_max_error_dist = ANSWER::Configurator::GetInstance()
        .GetParamValue("pallet_docking", "st_dekf_param", "d_max_error_dist")
        .convert<double>();
    st_param.st_pallet_docking_param.st_dekf_param.d_max_error_deg = ANSWER::Configurator::GetInstance()
        .GetParamValue("pallet_docking", "st_dekf_param", "d_max_error_deg")
        .convert<double>();
    st_param.st_pallet_docking_param.st_dekf_param.d_cvg_cov_d_thr = ANSWER::Configurator::GetInstance()
        .GetParamValue("pallet_docking", "st_dekf_param", "d_cvg_cov_d_thr")
        .convert<double>();
    st_param.st_pallet_docking_param.st_dekf_param.d_cvg_cov_a_thr = ANSWER::Configurator::GetInstance()
        .GetParamValue("pallet_docking", "st_dekf_param", "d_cvg_cov_a_thr")
        .convert<double>();
    st_param.st_pallet_docking_param.st_dekf_param.d_cov_Q0_d_weight = ANSWER::Configurator::GetInstance()
        .GetParamValue("pallet_docking", "st_dekf_param", "d_cov_Q0_d_weight")
        .convert<double>();
    st_param.st_pallet_docking_param.st_dekf_param.d_cov_Q0_a_weight = ANSWER::Configurator::GetInstance()
        .GetParamValue("pallet_docking", "st_dekf_param", "d_cov_Q0_a_weight")
        .convert<double>();
    st_param.st_pallet_docking_param.st_dekf_param.d_cov_Qm_dd_weight = ANSWER::Configurator::GetInstance()
        .GetParamValue("pallet_docking", "st_dekf_param", "d_cov_Qm_dd_weight")
        .convert<double>();
    st_param.st_pallet_docking_param.st_dekf_param.d_cov_Qm_da_weight = ANSWER::Configurator::GetInstance()
        .GetParamValue("pallet_docking", "st_dekf_param", "d_cov_Qm_da_weight")
        .convert<double>();
    st_param.st_pallet_docking_param.st_dekf_param.d_cov_Qm_ad_weight = ANSWER::Configurator::GetInstance()
        .GetParamValue("pallet_docking", "st_dekf_param", "d_cov_Qm_ad_weight")
        .convert<double>();
    st_param.st_pallet_docking_param.st_dekf_param.d_cov_Qm_aa_weight = ANSWER::Configurator::GetInstance()
        .GetParamValue("pallet_docking", "st_dekf_param", "d_cov_Qm_aa_weight")
        .convert<double>();
    st_param.st_pallet_docking_param.st_dekf_param.d_cov_R0_d_weight = ANSWER::Configurator::GetInstance()
        .GetParamValue("pallet_docking", "st_dekf_param", "d_cov_R0_d_weight")
        .convert<double>();
    st_param.st_pallet_docking_param.st_dekf_param.d_cov_R0_a_weight = ANSWER::Configurator::GetInstance()
        .GetParamValue("pallet_docking", "st_dekf_param", "d_cov_R0_a_weight")
        .convert<double>();
    st_param.st_pallet_docking_param.st_dekf_param.d_cov_Rm_dd_weight = ANSWER::Configurator::GetInstance()
        .GetParamValue("pallet_docking", "st_dekf_param", "d_cov_Rm_dd_weight")
        .convert<double>();
    st_param.st_pallet_docking_param.st_dekf_param.d_cov_Rm_da_weight = ANSWER::Configurator::GetInstance()
        .GetParamValue("pallet_docking", "st_dekf_param", "d_cov_Rm_da_weight")
        .convert<double>();
    st_param.st_pallet_docking_param.st_dekf_param.d_cov_Rm_ad_weight = ANSWER::Configurator::GetInstance()
        .GetParamValue("pallet_docking", "st_dekf_param", "d_cov_Rm_ad_weight")
        .convert<double>();
    st_param.st_pallet_docking_param.st_dekf_param.d_cov_Rm_aa_weight = ANSWER::Configurator::GetInstance()
        .GetParamValue("pallet_docking", "st_dekf_param", "d_cov_Rm_aa_weight")
        .convert<double>();

    // wingbody docking
    st_param.st_wingbody_docking_param.n_delay_msec = ANSWER::Configurator::GetInstance()
        .GetParamValue("wingbody_docking", "n_delay_msec")
        .convert<int>();
    st_param.st_wingbody_docking_param.d_robot2sensor_x_m = ANSWER::Configurator::GetInstance()
        .GetParamValue("wingbody_docking", "d_robot2sensor_x_m")
        .convert<double>();
    st_param.st_wingbody_docking_param.d_robot2sensor_y_m = ANSWER::Configurator::GetInstance()
        .GetParamValue("wingbody_docking", "d_robot2sensor_y_m")
        .convert<double>();
    st_param.st_wingbody_docking_param.d_robot2sensor_deg = ANSWER::Configurator::GetInstance()
        .GetParamValue("wingbody_docking", "d_robot2sensor_deg")
        .convert<double>();
    st_param.st_wingbody_docking_param.st_dekf_param.n_max_buffer_size = ANSWER::Configurator::GetInstance()
        .GetParamValue("wingbody_docking", "st_dekf_param", "n_max_buffer_size")
        .convert<int>();
    st_param.st_wingbody_docking_param.st_dekf_param.d_max_error_dist = ANSWER::Configurator::GetInstance()
        .GetParamValue("wingbody_docking", "st_dekf_param", "d_max_error_dist")
        .convert<double>();
    st_param.st_wingbody_docking_param.st_dekf_param.d_max_error_deg = ANSWER::Configurator::GetInstance()
        .GetParamValue("wingbody_docking", "st_dekf_param", "d_max_error_deg")
        .convert<double>();
    st_param.st_wingbody_docking_param.st_dekf_param.d_cvg_cov_d_thr = ANSWER::Configurator::GetInstance()
        .GetParamValue("wingbody_docking", "st_dekf_param", "d_cvg_cov_d_thr")
        .convert<double>();
    st_param.st_wingbody_docking_param.st_dekf_param.d_cvg_cov_a_thr = ANSWER::Configurator::GetInstance()
        .GetParamValue("wingbody_docking", "st_dekf_param", "d_cvg_cov_a_thr")
        .convert<double>();
    st_param.st_wingbody_docking_param.st_dekf_param.d_cov_Q0_d_weight = ANSWER::Configurator::GetInstance()
        .GetParamValue("wingbody_docking", "st_dekf_param", "d_cov_Q0_d_weight")
        .convert<double>();
    st_param.st_wingbody_docking_param.st_dekf_param.d_cov_Q0_a_weight = ANSWER::Configurator::GetInstance()
        .GetParamValue("wingbody_docking", "st_dekf_param", "d_cov_Q0_a_weight")
        .convert<double>();
    st_param.st_wingbody_docking_param.st_dekf_param.d_cov_Qm_dd_weight = ANSWER::Configurator::GetInstance()
        .GetParamValue("wingbody_docking", "st_dekf_param", "d_cov_Qm_dd_weight")
        .convert<double>();
    st_param.st_wingbody_docking_param.st_dekf_param.d_cov_Qm_da_weight = ANSWER::Configurator::GetInstance()
        .GetParamValue("wingbody_docking", "st_dekf_param", "d_cov_Qm_da_weight")
        .convert<double>();
    st_param.st_wingbody_docking_param.st_dekf_param.d_cov_Qm_ad_weight = ANSWER::Configurator::GetInstance()
        .GetParamValue("wingbody_docking", "st_dekf_param", "d_cov_Qm_ad_weight")
        .convert<double>();
    st_param.st_wingbody_docking_param.st_dekf_param.d_cov_Qm_aa_weight = ANSWER::Configurator::GetInstance()
        .GetParamValue("wingbody_docking", "st_dekf_param", "d_cov_Qm_aa_weight")
        .convert<double>();
    st_param.st_wingbody_docking_param.st_dekf_param.d_cov_R0_d_weight = ANSWER::Configurator::GetInstance()
        .GetParamValue("wingbody_docking", "st_dekf_param", "d_cov_R0_d_weight")
        .convert<double>();
    st_param.st_wingbody_docking_param.st_dekf_param.d_cov_R0_a_weight = ANSWER::Configurator::GetInstance()
        .GetParamValue("wingbody_docking", "st_dekf_param", "d_cov_R0_a_weight")
        .convert<double>();
    st_param.st_wingbody_docking_param.st_dekf_param.d_cov_Rm_dd_weight = ANSWER::Configurator::GetInstance()
        .GetParamValue("wingbody_docking", "st_dekf_param", "d_cov_Rm_dd_weight")
        .convert<double>();
    st_param.st_wingbody_docking_param.st_dekf_param.d_cov_Rm_da_weight = ANSWER::Configurator::GetInstance()
        .GetParamValue("wingbody_docking", "st_dekf_param", "d_cov_Rm_da_weight")
        .convert<double>();
    st_param.st_wingbody_docking_param.st_dekf_param.d_cov_Rm_ad_weight = ANSWER::Configurator::GetInstance()
        .GetParamValue("wingbody_docking", "st_dekf_param", "d_cov_Rm_ad_weight")
        .convert<double>();
    st_param.st_wingbody_docking_param.st_dekf_param.d_cov_Rm_aa_weight = ANSWER::Configurator::GetInstance()
        .GetParamValue("wingbody_docking", "st_dekf_param", "d_cov_Rm_aa_weight")
        .convert<double>();

    return st_param;
}

}

}  // namespace NVFR
