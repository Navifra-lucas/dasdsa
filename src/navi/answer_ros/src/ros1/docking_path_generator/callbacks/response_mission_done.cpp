#include "ros1/docking_path_generator/docking_path_generator_ros1.hpp"

#include "logger/logger.h"

namespace NVFR {

bool DockingPathGeneratorRos1::ResponseMissionDone(
    std_srvs::Trigger::Request  &req,
    std_srvs::Trigger::Response &res)
{
    {
        std::lock_guard<std::mutex> lock(mtx_mission_);
        if (n_mission_type_ != DockingInfo::CMD::NONE)
        {
            LOG_INFO("mission : {} -> {}",
                n_mission_type_,
                DockingInfo::CMD::NONE);
            n_mission_type_ = DockingInfo::CMD::NONE;
            if (o_mission_ptr_)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(100UL));
                o_mission_ptr_->Terminate();
            }
        }
    }

    res.success = true;

    return res.success;

}

} // namespace NVFR
