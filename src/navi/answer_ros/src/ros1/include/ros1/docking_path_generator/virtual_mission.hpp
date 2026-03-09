#ifndef VIRTUAL_MISSION_HPP_
#define VIRTUAL_MISSION_HPP_

#include <memory>

#include "utils/data_manager.hpp"
#include "utils/publish_callback.hpp"
#include "utils/loop_thread_handler.hpp"
#include "utils/pose.hpp"
#include "utils/common_math.hpp"
#include "utils/motion_planner_calculator.hpp"

#include "motion_controller/docking/docking.hpp"

#include "ros1/docking_path_generator/dpg_param.hpp"
#include "ros1/docking_path_generator/publish_manager.hpp"
#include "ros1/docking_path_generator/docking_nodes.hpp"
#include "ros1/docking_path_generator/smart_path_generator.hpp"

namespace NVFR {

class VirtualMission
{
public:

    VirtualMission(
        bool b_local,
        const Pose& o_robot2sensor,
        const Pose& o_map2object,
        const Pose& o_object2goal,
        const Pose& o_fNode,
        const DPG_Param_t& st_dpg_param);
    virtual ~VirtualMission();

    DockingNodes_t GetDockingNodes() const;

    void SetRobotPose(const Pose& o_robot_pose);
    bool SetObjectPose(const Pose& o_object_pose);

    void Terminate();
    void Stop();

protected:
    // param
    DataManager<DPG_Param_t> st_dpg_param_;

    // data
    const bool b_local_;
    const Pose o_fNode_;

    // thread handler
    LoopThread o_loop_thread_;

    // update status
    bool b_update_;
    bool b_initial_;
    mutable std::mutex mtx_;

    // docking obj
    std::shared_ptr<Docking> o_docking_ptr_;

    // current robot pose based on map coordinate
    DataManager<Pose> o_robot_pose_;

    // func
    bool DoFirstDocking();
    void ExecuteDocking();

private:

};

}  // namespace NVFR

#endif
